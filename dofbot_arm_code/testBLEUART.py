import asyncio
import sys
from itertools import count, takewhile
from typing import Iterator

from bleak import BleakClient, BleakScanner
from bleak.backends.characteristic import BleakGATTCharacteristic
from bleak.backends.device import BLEDevice
from bleak.backends.scanner import AdvertisementData

from numpy import interp
import smbus
import time
from Arm_Lib import Arm_Device

import cv2 as cv
from cvlib.object_detection import YOLO

import threading
import signal
import ImageTransferService

Arm = Arm_Device()
time.sleep(.1)

Arm.Arm_serial_servo_write6(90, 90, 0, 90, 90, 0, 1000)
offset = -99999
grabbing = 0
toggle = 0
hold_position = 0
last_heading = 0

time.sleep(1)

UART_SERVICE_UUID = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
UART_RX_CHAR_UUID = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
UART_TX_CHAR_UUID = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"


# Aligns robotic arm heading with heading of sensor
def get_offset(reading, offset):
    offset_val = (reading + offset) % 360
    if offset_val > 180:
        offset_val -= 360
    
    return offset_val

# TIP: you can get this function and more from the ``more-itertools`` package.
def sliced(data: bytes, n: int) -> Iterator[bytes]:
    """
    Slices *data* into chunks of size *n*. The last slice may be smaller than
    *n*.
    """
    return takewhile(len, (data[i : i + n] for i in count(0, n)))

async def uart_terminal():
    """This is a simple "terminal" program that uses the Nordic Semiconductor
    (nRF) UART service. It reads from stdin and sends each line of data to the
    remote device. Any data received from the device is printed to stdout.
    """

    def match_nus_uuid(device: BLEDevice, adv: AdvertisementData):
        # This assumes that the device includes the UART service UUID in the
        # advertising data. This test may need to be adjusted depending on the
        # actual advertising data supplied by the device.
        if UART_SERVICE_UUID.lower() in adv.service_uuids:
            return True

        return False

    device = await BleakScanner.find_device_by_filter(match_nus_uuid)

    if device is None:
        print("no matching device found, you may need to edit match_nus_uuid().")
        sys.exit(1)

    def handle_disconnect(_: BleakClient):
        print("Device was disconnected, goodbye.")
        # cancelling all tasks effectively ends the program
        for task in asyncio.all_tasks():
            task.cancel()
        global exit_flag
        exit_flag = True

    def handle_rx(_: BleakGATTCharacteristic, data: bytearray):
        global grabbing, offset, toggle, hold_position, last_heading
        print("received:", data)
        data = data.decode('utf-8')
        data = data.split(",")
        
        roll = float(data[0])
        pitch = float(data[1])
        heading = float(data[2])
        grab = float(data[3])
        btn = float(data[4])
        
        # if (grab == 69):
        #     if (grabbing == 0):
        #         grabbing = 1
        #     else:
        #         grabbing = 0
        if (btn == 1):
            toggle = 1

        elif (btn == 2):
            Arm.Arm_serial_servo_write6(last_heading, 135, 0, 0, 90, 0, 1000)
            hold_position = not hold_position

        elif (btn == 3):
            offset = -99999
        if (offset == -99999):
            offset = 90 - heading
        
        offset_heading = get_offset(heading, offset)
        heading = offset_heading
        
        if (heading < 0):
            if (heading > -90):
                heading = 0
            else:
                heading = 180
	
        if (roll < -90):
                roll = -90
        if (roll > 90):
                roll = 90

        if (hold_position == 0):
            last_heading = heading
            #pitch above 90 degrees, bend servo 3 only    
            if (pitch <= 0):
                Arm.Arm_serial_servo_write6(heading, 90, abs(pitch), 90, roll+90, 0, 500)
                
            #pitch below 90 degrees, bend servo 2 and 4, servo 3 stays at 90 degrees
            if (pitch > 0):
                ang2 = pitch + 90
                ang4 = 75 - pitch
                if (ang2 > 120):
                    ang2 = 120
                if (ang4 < 0):
                    ang4 = 0
                    ang2 = interp(pitch, [75, 90], [120, 90])
                
                Arm.Arm_serial_servo_write6(heading, ang2, 0, ang4, roll+90, 0, 500)

    async with BleakClient(device, disconnected_callback=handle_disconnect) as client:
        await client.start_notify(UART_TX_CHAR_UUID, handle_rx)

        capture_thread = threading.Thread(target=capture_frames)
        process_thread = threading.Thread(target=process_frames)

        capture_thread.start()
        process_thread.start()

        print("Connected, start typing and press ENTER...")

        loop = asyncio.get_running_loop()
        nus = client.services.get_service(UART_SERVICE_UUID)
        rx_char = nus.get_characteristic(UART_RX_CHAR_UUID)

        while not exit_flag:
            # This waits until you type a line and press ENTER.
            # A real terminal program might put stdin in raw mode so that things
            # like CTRL+C get passed to the remote device.
            data = await loop.run_in_executor(None, sys.stdin.buffer.readline)

            # data will be empty on EOF (e.g. CTRL+D on *nix)
            if not data:
                break

            # some devices, like devices running MicroPython, expect Windows
            # line endings (uncomment line below if needed)
            # data = data.replace(b"\n", b"\r\n")

            # Writing without response requires that the data can fit in a
            # single BLE packet. We can use the max_write_without_response_size
            # property to split the data into chunks that will fit.

            for s in sliced(data, rx_char.max_write_without_response_size):
                await client.write_gatt_char(rx_char, s, response=False)

            print("sent:", data)

capture = None
exit_flag = False
camera_thread = None
capture_thread = None
process_thread = None
frame = None
lock = threading.Lock()

host = '192.168.18.211'
#host = '192.168.11.17'
RemoteDisplay = ImageTransferService.ImageTransferService(host)

def handle_sigint(signal_number, frame):
    print("Exiting...")
    global exit_flag
    exit_flag = True
    
    if capture is not None:
        capture.release()
    cv.destroyAllWindows()
    
    sys.exit(0)

def count_fps(start_time, fps_total, frame_count):
    cur_time = time.time()
    elapsed = cur_time - start_time
    fps = 1/elapsed
    fps_total += fps
    frame_count += 1
    avg_fps = fps_total/frame_count
    return cur_time, fps_total, frame_count, avg_fps
    
def capture_frames():
    global capture, frame
    capture = cv.VideoCapture(0)
    
    while not exit_flag:
        ret, new_frame = capture.read()
        if not ret:
            break
        with lock:
            frame = new_frame 

def process_frames():
    start_time = time.time()
    fps_total = 0
    frame_count = 0
    current_objects = []
    target_obj = ''
    obj_index = 0
    global frame, toggle
    yolo = YOLO('yolov4-tiny-custom_best.weights', 'yolov4-tiny-test.cfg', 'obj.names')
    while not exit_flag:
        if frame is not None:
            with lock:
                current_frame = frame.copy()
                
            bbox, label, conf = yolo.detect_objects(current_frame)
            yolo.draw_bbox(current_frame, bbox, label, conf, write_conf=True)

            if hold_position:
                for x in label:
                    if x not in current_objects:
                        current_objects.append(x)
                if current_objects:
                    if (toggle == 1):
                        obj_index = (obj_index + 1) % len(current_objects)
                        target_obj = current_objects[obj_index]
                        i = 0
                        while (current_objects[obj_index] not in label):
                            obj_index = (obj_index + 1) % len(current_objects)
                            target_obj = current_objects[obj_index]
                            i+=1
                            if i>=len(current_objects):
                                target_obj = ''
                                break

                        print("switch target")
                        toggle = 0
                    
                    targ_txt = "Target: {}".format(target_obj)
                    cv.putText(current_frame, targ_txt, (30, 30), cv.FONT_HERSHEY_COMPLEX_SMALL, 1, (0,0,255), 1)
            else:
                current_objects.clear()
                
            print(obj_index, len(current_objects))
            print(current_objects)
            
            start_time, fps_total, frame_count, avg_fps = count_fps(start_time, fps_total, frame_count)
            fps_text = "FPS: {:.2f}".format(avg_fps)
            cv.putText(current_frame, fps_text, (30, 430), cv.FONT_HERSHEY_COMPLEX_SMALL, 1, (0,0,255), 1)

            RemoteDisplay.sendImage(current_frame)
            #cv.imshow('video', current_frame)
        
            #if cv.waitKey(1) & 0xFF == ord('q'):
                #break
    if capture:
        capture.release()
    cv.destroyAllWindows()

# def camera():
#     start_time = time.time()
#     frame_count = 0
#     fps_total = 0

#     yolo = YOLO('yolov4-tiny-custom_best.weights', 'yolov4-tiny-test.cfg', 'obj.names')
    
#     global capture
#     capture = cv.VideoCapture(0)
    
#     while not exit_flag:
#         ret, frame = capture.read()
#         if not ret:
#             break
        
#         bbox, label, conf = yolo.detect_objects(frame)
#         yolo.draw_bbox(frame, bbox, label, conf, write_conf=True)

#         start_time, fps_total, frame_count, avg_fps = count_fps(start_time, fps_total, frame_count)
#         fps_text = "FPS: {:.2f}".format(avg_fps)
#         cv.putText(frame, fps_text, (30, 430), cv.FONT_HERSHEY_COMPLEX_SMALL, 1, (0,0,255), 1)

#         cv.imshow('video', frame)
        
#         if cv.waitKey(1) & 0xFF == ord('q'):
#             break
    
#     if capture:
#         capture.release()
#     cv.destroyAllWindows()

if __name__ == "__main__":
    signal.signal(signal.SIGINT, handle_sigint)
    
    try:
        asyncio.run(uart_terminal())
    except asyncio.CancelledError:
        # task is cancelled on disconnect, so we ignore this error
        pass
    
    finally:
        if capture is not None:
            capture.release()
        cv.destroyAllWindows()
        if capture_thread is not None:
            capture_thread.join()
        if process_thread is not None:
            process_thread.join()
