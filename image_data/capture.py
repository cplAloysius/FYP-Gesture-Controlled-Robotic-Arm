import cv2
import time
count = 0
maxFrames = 10

cap=cv2.VideoCapture(0)

while count < maxFrames:
    ret, img = cap.read()
    img = cv2.resize(img, (640,480))
    img2 = img.copy()
    img2 = cv2.putText(img2, str(count), (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0))
    cv2.imshow("test window", img2)
    if cv2.waitKey(1) & 0xFF == ord(' '):
    	cv2.imwrite("/home/pi/FYP-Gesture-Controlled-Robotic-Arm/train_img/extra imgs/nightcolours_%d.jpg" %count, img)
    	count += 1
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
cap.release()
cv2.destroyAllWindows()
