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

from identify_target import identify_GetTarget

arm_service = identify_GetTarget()

Arm = Arm_Device()
time.sleep(.1)

Arm.Arm_serial_servo_write6(90, 90, 0, 90, 90, 0, 1000)
offset = -99999
grabbing = 0
toggle = 0
hold_position = 0
last_heading = 0


capture = None
exit_flag = False
camera_thread = None
capture_thread = None
process_thread = None
frame = None
lock = threading.Lock()
holding_block = 0

host = '192.168.18.211'
#host = '192.168.11.17'
RemoteDisplay = ImageTransferService.ImageTransferService(host)
print("Redis server running: ", RemoteDisplay.ping())

time.sleep(1)

address1 = "F3:2F:38:4B:9E:7E"
address2 = "F3:D6:58:89:7D:CF"

UART_SERVICE_UUID = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
UART_TX_CHAR_UUID = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"


# Aligns robotic arm heading with heading of sensor
def get_offset(reading, offset):
    offset_val = (reading + offset) % 360
    if offset_val > 180:
        offset_val -= 360
    
    return offset_val

async def uart_terminal(device, dev_num):

    def handle_disconnect(_: BleakClient):
        print("Device {} was disconnected, goodbye.".format(dev_num))
        # cancelling all tasks effectively ends the program
        for task in asyncio.all_tasks():
            task.cancel()
        global exit_flag
        exit_flag = True

    def handle_rx(_: BleakGATTCharacteristic, data: bytearray):
        global offset, grabbing, last_heading, toggle, hold_position
        if dev_num == 1:
            #global grabbing, last_heading
            print("received:", data)
            data = data.decode('utf-8')
            data = data.split(",")
            
            roll = float(data[0])
            pitch = float(data[1])
            heading = float(data[2])
            claw = float(data[3])
            # btn = 0
            
            # if (btn == 1):
            #     toggle = 1

            # elif (btn == 2 and not holding_block):
            #     Arm.Arm_serial_servo_write6(last_heading, 135, 0, 0, 90, 0, 1000)
            #     hold_position = not hold_position

            # elif (btn == 3):
            #     offset = -99999
            if (offset == -99999):
                offset = 90 - heading

            if hold_position and claw == 69:
                grabbing = 1
            
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
            
            if holding_block and not grabbing:
                Arm.Arm_serial_servo_write(1, heading, 500)
        
        else:
            #global toggle, hold_position
            print("button:", data)
            data = data.decode('utf-8')
            btn = float(data)
            print(btn)
            
            if (btn == 1):
                toggle = 1

            # elif (btn == 2 and not holding_block):
            #     Arm.Arm_serial_servo_write6(last_heading, 135, 0, 0, 90, 0, 1000)
            #     hold_position = not hold_position

            elif (btn == 3):
                offset = -99999

    async with BleakClient(device, disconnected_callback=handle_disconnect) as client:
        await client.start_notify(UART_TX_CHAR_UUID, handle_rx)

        if dev_num == 1:
            capture_thread = threading.Thread(target=capture_frames)
            process_thread = threading.Thread(target=process_frames)

            capture_thread.start()
            process_thread.start()

        print("Connected to device {}, receiving data...".format(dev_num))

        while True:
            await asyncio.sleep(1)

async def main():
    device1 = await BleakScanner.find_device_by_address(address1)
    if device1 is None:
        print("device 1 not found")
        sys.exit(1)

    device2 = await BleakScanner.find_device_by_address(address2)
    if device2 is None:
        print("device 2 not found")
        sys.exit(1)

    await asyncio.gather(
        uart_terminal(device1, 1),
        uart_terminal(device2, 2)
    )

def handle_sigint(signal_number, frame):
    print(" Exiting...")
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
    cntr = None
    global frame, toggle, grabbing, holding_block, hold_position, last_heading
    yolo = YOLO('yolov4-tiny-custom_best.weights', 'yolov4-tiny-test.cfg', 'obj.names')
    while not exit_flag:
        if frame is not None:
            with lock:
                current_frame = frame.copy()

            bbox, label, conf = yolo.detect_objects(current_frame)
            yolo.draw_bbox(current_frame, bbox, label, conf, write_conf=True)

            if holding_block:
                if grabbing:
                    cntr = None
                    last_heading  = Arm.Arm_serial_servo_read(1)
                    arm_service.xy = [last_heading, 135]
                    threading.Thread(target=arm_service.target_run, args=(cntr,)).start()
                    if arm_service.move_status:
                        time.sleep(4)
                    grabbing = 0
                    hold_position = 0
                    holding_block = 0

            elif hold_position and not holding_block:
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
                        toggle = 0
                    
                    targ_txt = "Target: {}".format(target_obj)
                    cv.putText(current_frame, targ_txt, (30, 30), cv.FONT_HERSHEY_COMPLEX_SMALL, 1, (0,0,255), 1)

                    if grabbing:
                        if target_obj in label:
                            i = label.index(target_obj)
                            centre = (int((bbox[i][0] + bbox[i][2])/2), int((bbox[i][1] + bbox[i][3])/2))
                            scale_x, scale_y = scaling(centre[0], centre[1])
                            cntr = (round((((centre[0]*scale_x) - 320) / 4000), 5), round(((480 - (centre[1]*scale_y)) / 3000) * 0.8+0.19, 5))
                        else:
                            cntr = None

                        last_heading  = Arm.Arm_serial_servo_read(1)
                        arm_service.xy = [last_heading, 135]
                        threading.Thread(target=arm_service.target_run, args=(cntr,)).start()
                        if arm_service.move_status:
                            time.sleep(6)
                            if not arm_service.exceed_angle:
                                holding_block = 1
                            else:
                                arm_service.exceed_angle = 0
                        grabbing = 0
            else:
                current_objects.clear()
            
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

def scaling(x, y):
    scale_x = 1
    scale_y = 1
    if y<240:
        scale_y = y/240
    if x<320:
        scale_x = x/320
    elif x>320:
        scale_x = (x-320)/3200 + 1

    return scale_x, scale_y


if __name__ == "__main__":
    signal.signal(signal.SIGINT, handle_sigint)
    
    try:
        asyncio.run(main())
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
