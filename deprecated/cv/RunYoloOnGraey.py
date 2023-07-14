from ultralytics import YOLO
from ultralytics.yolo.v8.detect.predict import DetectionPredictor
import cv2
#loading the model from the folder
model = YOLO("smallestmodel.pt")
#importing the video
importedVid = cv2.VideoCapture("eger.mp4")
if not importedVid.isOpened():
 print("Cannot open camera")
 exit()
while True:
    ret, frame = importedVid.read()
    frame = cv2.resize(frame, (640,480))
    # actually runs the model on the image frame, returns a list called results of size 1 (for some reason) with all results in it
    #source is the source of the image, show determines whether or not to show the frame ( i use imshow since it's easier and i can draw on it)
    #verbose is whether or not to show logs (see image on doc for what it looks like)
    results = model.predict(source = frame, show = False, verbose = False)
    # extracts the boxes from the results list, then turns it into numpy to be easier to work with
    boxes = results[0].boxes.data.numpy()
    #although results is a size 1 array, boxes actually has multiple values so we have to enumerate it and run for each box inside of the boxes array
    for idx in enumerate(boxes):
        #extracting idx'th box
        boxInfo = boxes[idx]
        #boxInfo comes with 6 values: [x1, x2, y1, y2, confidence (from 0 to 1), label (0 or 1 in this case)]
        if(boxInfo[5]==0): 
            #since x1, x2, y1, and y2 are decimals, we have to int it.
            frame  = cv2.rectangle(frame, (int(boxInfo[0]),int(boxInfo[1])), (int(boxInfo[2]),int(boxInfo[3])), (0,0,255), 1)
        elif(boxInfo[5]==1): 
            frame  = cv2.rectangle(frame, (int(boxInfo[0]),int(boxInfo[1])), (int(boxInfo[2]),int(boxInfo[3])), (0,255,0), 1)
    cv2.imshow("frame", frame)
    if cv2.waitKey(5) == ord('q'):
        break
importedVid.release()
