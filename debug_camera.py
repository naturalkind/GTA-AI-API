import cv2
import time

# Wait for capture to start
time.sleep(1)

# Open the virtual camera
device_id = 10
camera = cv2.VideoCapture(device_id, cv2.CAP_V4L2)
#camera.set(cv2.CAP_PROP_CONVERT_RGB, 0)  # ← ЭТО ГЛАВНОЕ!

while True:
    ret, frame = camera.read()
    if ret:
        cv2.imshow('Virtual Camera', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

camera.release()
cv2.destroyAllWindows()

