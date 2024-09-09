import cv2
from cvlib.object_detection import YOLO
import time

yolo = YOLO('yolov4-tiny-custom_best.weights', 'yolov4-tiny-test.cfg', 'obj.names')
cap = cv2.VideoCapture(0)

def count_fps(start_time, fps_total, frame_count):
    cur_time = time.time()
    elapsed = cur_time - start_time
    fps = 1/elapsed
    fps_total += fps
    frame_count += 1
    avg_fps = fps_total/frame_count
    return cur_time, fps_total, frame_count, avg_fps

start_time = time.time()
fps_total = 0
frame_count = 0

while True:
    ret, frame = cap.read()
    bbox, label, conf = yolo.detect_objects(frame)
    yolo.draw_bbox(frame, bbox, label, conf, write_conf=True)

    start_time, fps_total, frame_count, avg_fps = count_fps(start_time, fps_total, frame_count)
    fps_text = "FPS: {:.2f}".format(avg_fps)
    cv2.putText(frame, fps_text, (30, 430), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1, (0,0,255), 1)

    cv2.imshow('img', frame)
    if cv2.waitKey(1) & 0xFF==ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
