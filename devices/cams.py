import cv2

camera_streams = []

def init_cam(cid, num):
    camera_streams.push([cid, cv2.VideoCapture(num)])

    if not camera_streams[-1][1].isOpened():
        print("err: cam " + cid + " failed to initialize")
        return 1

    return 0

def get_frame(cid):
    cam = None
    for i in camera_streams:
        if i[0] == cid:
            cam = i

    ret, frame = cam.read()

def show(f):
    cv2.imshow(f)

