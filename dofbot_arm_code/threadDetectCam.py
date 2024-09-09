import cv2
from cvlib.object_detection import YOLO
import threading
import time

yolo = YOLO('yolov4-tiny-custom_best.weights', 'yolov4-tiny-test.cfg', 'obj.names')

cap = cv2.VideoCapture(0)

start_time = time.time()
frame = None
lock = threading.Lock()

running = True

fps_total = 0
frame_count = 0

def count_fps(start_time, fps_total, frame_count):
    cur_time = time.time()
    elapsed = cur_time - start_time
    fps = 1/elapsed
    fps_total += fps
    frame_count += 1
    avg_fps = fps_total/frame_count
    return cur_time, fps_total, frame_count, avg_fps

def capture_frames():
    global frame, running
    while running:
        ret, new_frame = cap.read()
        if ret:
            with lock:
                frame = new_frame

def process_frames():
    global frame, running,start_time, frame_count, fps_total
    while running:
        if frame is not None:
            with lock:
                current_frame = frame.copy()

            bbox, label, conf = yolo.detect_objects(current_frame)
            yolo.draw_bbox(current_frame, bbox, label, conf, write_conf=True)

            start_time, fps_total, frame_count, avg_fps = count_fps(start_time, fps_total, frame_count)
            fps_text = "FPS: {:.2f}".format(avg_fps)
            cv2.putText(current_frame, fps_text, (30, 430), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1, (0,0,255), 1)

            cv2.imshow('img', current_frame)
            if cv2.waitKey(1) & 0xFF==ord('q'):
                running = False

capture_thread = threading.Thread(target=capture_frames)
process_thread = threading.Thread(target=process_frames)

capture_thread.start()
process_thread.start()

capture_thread.join()
process_thread.join()

cap.release()
cv2.destroyAllWindows()
