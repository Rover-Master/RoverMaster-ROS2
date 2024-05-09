import cv2

video = cv2.VideoCapture(0, cv2.CAP_V4L2)

if not video.isOpened():
    print("Failed to open camera.")
    exit(1)

video.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
video.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
video.set(cv2.CAP_PROP_BUFFERSIZE, 1)
