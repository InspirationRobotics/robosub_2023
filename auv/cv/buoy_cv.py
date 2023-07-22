"""
Description: Buoy CV
Author: Eesh Vij
"""

# import what you need from within the package

import time
import cv2
import numpy as np
from shapely.geometry import LineString
import math

class CV:
    """Buoy CV class, don't change the name of the class"""

    camera = "/auv/camera/videoOAKdRawForward"
    model = "buoy"

    def __init__(self, **config):
        """
        Init of the class,
        setup here everything that will be needed for the run fonction
        config is a dictionnary containing the config of the sub
        """
        self.midX = 320
        self.midY = 240
        print("[INFO] Buoy CV init")


    def process_frame(self, frame): #obselete method for cv detection instead of oak-d model
        # # frame_b, frame_g, frame_r = cv2.split(frame)
        # # frame_g = cv2.equalizeHist(frame_g)
        # # frame_b = cv2.equalizeHist(frame_b)
        # # frame_r = cv2.equalizeHist(frame_r)
        # # frame = cv2.merge((frame_b, frame_g, frame_r))
        # frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # frame = cv2.GaussianBlur(frame, (3,3), 0)
        # #frame = cv2.bilateralFilter(frame,9,75,75)
        # #ret, frame = cv2.threshold(frame,150,255,0)
        # #sobelx = cv2.Sobel(src=frame, ddepth=cv2.CV_64F, dx=1, dy=0, ksize=5) # Sobel Edge Detection on the X axis
        # #sobely = cv2.Sobel(src=frame, ddepth=cv2.CV_64F, dx=0, dy=1, ksize=5) # Sobel Edge Detection on the Y axis
        # #sobelxy = cv2.Sobel(src=frame, ddepth=cv2.CV_64F, dx=1, dy=1, ksize=5)
        # edges = cv2.Canny(frame, 50, 200, L2gradient=False) # Canny Edge Detection #0 200
        # contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        # for cnts, contour in enumerate(contours):
        #         area = cv2.contourArea(contour)
        #         if(area>500 and area<6000):
        #             rect = cv2.minAreaRect(contour)
        #             box = cv2.boxPoints(rect)
        #             box = np.intp(box)
        #             x, y, w, h = cv2.boundingRect(contour)
        #             cx = int((x+x+w)/2)
        #             cy = int((y+y+h)/2)
        #             cv2.rectangle(frame,(x,y),(x+w,y+h),(0,255,0),2)
        #             #cv2.drawContours(frame,[box],0,(255,0,0),2)
        return

    class point:
        def __init__(self, x, y):
            self.x = x
            self.y = y
            self.visited = [(self.x, self.y)]
        def visit(self, point):
            if((point.x, point.y) in self.visited):
                return False
            self.visited.append((point.x, point.y))
            return True
        def getPoint(self):
            return (self.x, self.y)
        def getPointInt(self):
            return [int(self.x), int(self.y)]
        
    class line:
        def __init__(self, point1, point2):
            self.slope = abs((point2.y - point1.y) / ((point2.x/point1.x)+0.0001)) # so we dont divide by 0
            self.midpoint = ((point2.x + point1.x)/2, (point2.y + point1.y)/2)
            self.length = math.hypot(point1.x - point2.x, point1.y - point2.y)
            self.point1 = point1
            self.point2 = point2

        def intersects(self, lineToCheck):
            line1 = LineString([self.point1.getPoint(), self.point2.getPoint()])
            line2 = LineString([lineToCheck.point1.getPoint(), lineToCheck.point2.getPoint()])
            int_pt = line1.intersection(line2)
            if int_pt == None:
                return None
            point_of_intersection = (int_pt.x, int_pt.y)
            return point_of_intersection

    def calculate_data(self, frame, target, detections):
        toReturn = {"frame": frame}
        if len(detections)==0:
            return toReturn
        correctRatios = {"A1": 1.0324, "A2": 0.3057, "E1": 1.1809, "E2": 1.5506, "board": 1} # based off sample footage with sub looking straight at them
        correctSizes = {"A1": 9000, "A2": 3700, "E1": 6308, "E2": 8200, "board": 31255, "dist": 2} #each one has its own pixel size at a set distance. # dist is in meters
        validDetections = {}
        detectedLabels = {}
        avgOffset = avgDist = c = 0
        targetCenter=boardCenter=boardDetect=None
        # first parse for valid dections and ignore duplicates/reflections
        for detection in detections:
            if detection.label in validDetections.keys():
                if detection.ymin<validDetections[detection.label].ymin:
                    continue #ignore reflection of same label, otherwise overwrite
            validDetections[detection.label] = detection
        # now parse all the valid detections
        #print("Reached 1")
        for detection in validDetections.values():
            label = detection.label
            x_length = abs(detection.xmax-detection.xmin)
            y_length = abs(detection.ymax-detection.ymin)
            xMid = detection.xmin+x_length/2
            yMid = detection.ymin+y_length/2
            if label != "board": 
                detectedLabels[label] = self.point(xMid, yMid) # x y is center of detection
                cv2.circle(frame, detectedLabels[label].getPointInt(), 5, (0,0,255),-1)
            else: 
                boardDetect = [xMid, yMid]
                cv2.circle(frame, (int(boardDetect[0]), int(boardDetect[1])), 5, (0,0,255),-1)
            if(not (detection.xmax>=630 or detection.xmin<=10 or detection.ymax>=470 or detection.ymin<=10)):
                # if not on edge of frame do area and ratio calculation
                area = x_length*y_length
                dist = (correctSizes["dist"]*correctSizes[label])/area
                avgDist+=dist
                xy_ratio = x_length/y_length
                avgOffset+= (correctRatios[label]/xy_ratio) #based off the idea that xy_ratio * z = correct_ratio and we average z for all detections
                c+=1 #counter for averaging
        #print("Reached 2")
        if c>0:
            avgOffset = avgOffset/c
            avgDist = avgDist/c
        #Forming lines between points
        testPoints = []
        lines = []
        for i in detectedLabels.values():
            testPoints.append(i)
        totalLines = (len(testPoints)*(len(testPoints)-1))/2
        nextPoint=0
        while(len(lines)<totalLines):
            nextPoint+=1
            for i in range(len(testPoints)):
                nextPoint = nextPoint%len(testPoints)
                if testPoints[i].visit(testPoints[nextPoint]) and testPoints[nextPoint].visit(testPoints[i]):
                    cv2.line(frame, testPoints[i].getPointInt(), testPoints[nextPoint].getPointInt(), (219, 112, 147), 2)
                    lines.append(self.line(testPoints[i], testPoints[nextPoint]))
        ######
        #Now calculating center of board based off lines
        #print("Reached 3")
        if len(lines)>=1:
            def getLength(line):
                return line.length
            lines.sort(reverse=True, key=getLength)
            if len(lines)>4:
                boardCenter = lines[0].intersects(lines[1])
            else:
                boardCenter = lines[0].midpoint
        if boardDetect != None and boardCenter != None:
                boardCenter = [boardDetect[i] if abs(v-boardDetect[i])>100 else v for i,v in enumerate(boardCenter)]
                boardCenter = [(boardCenter[0]+boardDetect[0])/2, (boardCenter[1]+boardDetect[1])/2] #averaging
        elif boardDetect != None:
            boardCenter = boardDetect
        if boardCenter != None:
            boardCenter = [0 if v is None else v for v in boardCenter]
            cv2.circle(frame, (int(boardCenter[0]), int(boardCenter[1])), 5, (255,255,255),-1)
        targetCenter = detectedLabels.get(target, None)
        if targetCenter!=None:
            targetCenter = targetCenter.getPointInt()
            cv2.circle(frame, targetCenter, 5, (0,255,0),-1)
        toReturn["frame"] = frame
        toReturn["offset"] = avgOffset
        toReturn["avgDist"] = round(avgDist,3)
        toReturn["targetCenter"] = targetCenter
        toReturn["boardCenter"] = boardCenter
        return toReturn

    def run(self, frame, target, oakd_data):
        """
        frame: the frame from the camera
        target: could be any type of information, for example the thing to look for
        oakd_data: only applies for oakd cameras, this is the list of detections

        Here should be all the code required to run the CV.
        This could be a loop, grabing frames using ROS, etc.
        """
        if oakd_data == None:
            return frame
        result = self.calculate_data(frame, target, oakd_data)
        # ratio is x/y so a the smaller the ratio the more askew you are yaw wise
        # if ratio is close to 1 then you are perfectly aligned
        print(result.get("targetCenter"))
        #print("[INFO] Buoy CV run")
        
        return {"lateral": 0, "forward": 0, "end": False}, result["frame"]


if __name__ == "__main__":
    # This is the code that will be executed if you run this file directly
    # It is here for testing purposes
    # you can run this file independently using: "python -m auv.cv.template_cv"

    # Create a CV object with arguments
    cv = CV(arg1="value1", arg2="value2")

    # here you can for example initialize your camera, etc
    cap = cv2.VideoCapture("/Users/eeshvij/Desktop/Code/robosub_2023/testing_data/buoyUSB.mp4")

    while True:
        # grab a frame
        ret, frame = cap.read()
        if not ret:
            cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            continue

        # run the cv
        result = cv.run(frame, "A1", None)

        # do something with the result
        #print(f"[INFO] {result}")

        # debug the frame
        cv2.imshow("frame", result[1])
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
        time.sleep(1/40)