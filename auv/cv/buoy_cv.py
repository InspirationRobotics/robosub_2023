"""
CV processing for the Buoy mission
"""

import time 
import cv2 
import numpy as np 
from shapely.geometry import LineString # Operations with geometry (in this case straight lines)
import math 
from copy import deepcopy # For duplicating nested data structures, so that all instances of objects are duplicated

class CV:
    """
    Class for running the Buoy mission
    """

    camera = "/auv/camera/videoOAKdRawForward" # Camera stream to use
    model = "buoytransdec" # ML model to run

    def __init__(self, **config):
        """
        Initialize the Buoy CV class

        Args:
            config: Mission specific parameters for running the mission
        """
        # Initialize variables
        self.midX = 320 
        self.midY = 240 
        self.prevYaw=0.6 
        self.prevStrafe = 1 
        self.prevRatio=1 
        self.step = 0 
        self.finished=False 
        self.target = None 
        self.targetSide = None 
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
        """
        Multi-purpose class for doing things with positional points
        """
        def __init__(self, x, y):
            self.x = x
            self.y = y
            self.visited = [(self.x, self.y)]

        def visit(self, point): 
            """
            Check if the sub has visited this point

            Args:
                point (float): Current position of the sub

            Returns:
                bool: Whether the point has been visted yet or not
            """
            if((point.x, point.y) in self.visited):
                return False
            self.visited.append((point.x, point.y)) # Add this point to the list of visited points
            return True
        
        def getPoint(self): 
            """Returns the current position (point) in (x, y) format"""
            return (self.x, self.y)
        
        def getPointInt(self): 
            """Returns the current position (poing) in (x, y) format where x and y are integers, not floats"""
            return [int(self.x), int(self.y)]
        
    class line:
        """
        Class for doing things with line segments
        """
        def __init__(self, point1, point2):
            """
            Initializes a line object between two points

            Args:
                point1: First endpoint of the line
                point2: Second endpoint of the line
            """
            self.slope = abs((point2.y - point1.y) / ((point2.x/point1.x)+0.0001)) # Slope of the line; taking precautions to not divide by 0
            self.midpoint = ((point2.x + point1.x)/2, (point2.y + point1.y)/2) # Calculate midpoint
            self.length = math.hypot(point1.x - point2.x, point1.y - point2.y) # Calculate length of the line using (Euclidean) distance formula
            self.point1 = point1
            self.point2 = point2

        def intersects(self, lineToCheck):
            """
            Checks if this line intersects with another line

            Args:
                linetoCheck (lineString): The line to check for intersection

            Returns:
                tuple: Point of intersection in (x, y) format
            """
            line1 = LineString([self.point1.getPoint(), self.point2.getPoint()])
            line2 = LineString([lineToCheck.point1.getPoint(), lineToCheck.point2.getPoint()])
            int_pt = line1.intersection(line2) # Find the intersection
            if int_pt == None:
                return None
            point_of_intersection = (int_pt.x, int_pt.y)
            return point_of_intersection

    def calculate_data(self, frame, target, detections):
        """
        Calculates data about the buoy from frame and detections; the detections are taken from the output of the ML model

        Args:
            frame (numpy.ndarray): Frame from the camera
            target: The target (information to look for)
            detections: The list of the detections

        Returns:
            list: [frame, averaged error of the x/y ratio relative to the predetermined values, average distance from the buoy, 
            which side of the buoy we are on, target's center, center of the board, side of target we are on]
        """

        """
        For clarification -- the board represents the actual buoy. There are characters on the "board" -- these are the two Earth and two Abydos characters.
        We want to bump either the Abydos ones or the Earth ones.
        """
        """
        The ratio is x/y so the smaller the ratio the more askew you are from the perspective of orientation. The smaller the ratio in other words, the
        more you have to yaw; if the ratio is close to 1 then that means perfect alignment
        """
        toReturn = {} # Initialize dictionary to store data

        # Define a dictionary containing correct ratios (length/width) for different labels(characters)
        correctRatios = {"abydos1": 1.1154, "abydos2": 0.2881, "earth1": 1.186, "earth2": 1.5476, "board": 1.0149} # Based of sample footage from head-on look
        
        # Correct sizes for different characters
        correctSizes = {"abydos1": 9000, "abydos2": 3700, "earth1": 6308, "earth2": 8200, "board": 31255, "dist": 3} # Each character has its own size at different distances (distance in meters).
        
        validDetections = {} # Dict for storing valid detections without duplicates
        detectedLabels = {} # Stores detected characters with their center points
        avgOffset = avgDist = c = 0 # Initialize variables for averaging offset and distance
        targetCenter=boardCenter=boardDetect=side=None # Initialize variables for target and board center points
        
        # First parse for valid detections, for deleting/ignoring duplicates/reflections
        for detection in detections:
            if detection.label in validDetections.keys(): # If the detection label is already in validDetections
                if detection.ymin < validDetections[detection.label].ymin: 
                    continue # Ignore reflection of same label, otherwise overwrite
            validDetections[detection.label] = detection
        
        # Now parse all the valid detections
        for detection in validDetections.values():
            label = detection.label
            # Calculate the length and width of the detection
            x_length = abs(detection.xmax-detection.xmin)
            y_length = abs(detection.ymax-detection.ymin)
            # Find the midpoint of the detection (also known as the center)
            xMid = detection.xmin+x_length/2
            yMid = detection.ymin+y_length/2

            # If the label is not board (so Abydos or Earth)
            if label != "board": 
                detectedLabels[label] = self.point(xMid, yMid) # Since label is valid, append it to the detectedLabels list: (x, y) is the center of detection
                cv2.circle(frame, detectedLabels[label].getPointInt(), 5, (0,0,255),-1) # Draw a circle with the correct label around the detection
            else: 
                boardDetect = [xMid, yMid]
                cv2.circle(frame, (int(boardDetect[0]), int(boardDetect[1])), 5, (0,0,255),-1) # Draw a circle without a label to designate the board
            
            if(not (detection.xmax>=630 or detection.xmin<=10 or detection.ymax>=470 or detection.ymin<=10)):
                # If not on edge of the frame, then do the area and ratio calculation 
                area = x_length*y_length
                dist = (correctSizes["dist"]*correctSizes[label])/area # Normalize the distance of the sub from the buoy
                avgDist+=dist # Add normalized distance to average distance -> avgDist is CURRENTLY the summation of distance; this will be averaged out by the number of detections later
                xy_ratio = x_length/y_length
                cv2.putText(frame, str(round(xy_ratio,4)), (int(xMid) + 10, int(yMid) + 20), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
                
                # avgOffset RIGHT NOW is the summation of all of the errors; this will be divided by the number of detections to get the average offset
                avgOffset+= (correctRatios[label]/xy_ratio)  
                c+=1 # Increment counter for averaging
        
        # Average out the offset
        if c>0:
            avgOffset = avgOffset/c
            avgDist = avgDist/c

        #Forming lines between points
        testPoints = []
        lines = []
        for i in detectedLabels.values():
            testPoints.append(i)

        # Formula for finding the number of possible "two point" combinations with a given number of points; this effectively finds the number of 
        # total possible lines without duplicates.
        totalLines = (len(testPoints)*(len(testPoints)-1))/2 
        nextPoint=0

        # We then draw all of these possible lines, by visiting two endpoints and drawing a point between them
        while(len(lines)<totalLines):
            nextPoint+=1
            for i in range(len(testPoints)):
                nextPoint = nextPoint%len(testPoints)
                if testPoints[i].visit(testPoints[nextPoint]) and testPoints[nextPoint].visit(testPoints[i]):
                    cv2.line(frame, testPoints[i].getPointInt(), testPoints[nextPoint].getPointInt(), (219, 112, 147), 2)
                    lines.append(self.line(testPoints[i], testPoints[nextPoint]))
        
        # print("Reached 3")

        # Now calculating the center of the board based off the lines drawn on the board
        if len(lines)>=1:
            # Sort the number of lines based on reversed order (from largest to smallest)
            lines.sort(reverse=True, key=lambda p: p.length)

            # If there are more than 4 lines, that means that we are looking at characters, and the board (the buoy itself)
            if len(lines)>4:
                # If there are more than 4 lines, take the two largest lines and find whether they intersect (these are the diagonals of the square buoy)
                boardCenter = lines[0].intersects(lines[1])

                # Find the leftmost line and rightmost line out of the two largest lines
                leftLine = min(lines, key=lambda p: p.midpoint[0])
                rightLine = max(lines, key=lambda p: p.midpoint[0])

                # Discriminate the side of the board (buoy) we are on based on which line is longer
                if leftLine.length>rightLine.length:
                    side="left"
                elif leftLine.length<rightLine.length:
                    side="right"
                else:
                    side="center"

                # Check which characters are the two lowest characters and return those
                lowerDetects = {}
                temp = testPoints.copy()
                # Sorts from lowest targets on the screen to highest (this is because the frame orientation is backward so those with the highest y-coordinates
                # are at the bottommost part of the screen) -> this sorts from high y-coordinate to low y-coordinate
                temp.sort(reverse=True, key=lambda p: p.y)
                temp = temp[:2] # Get the two lowest targets
                temp.sort(key=lambda p: p.x) # Sorts from leftmost x-coordinate to rightmost x-coordinate, so position 0 is left detection, position 1 is right detection
                label_keys = list(detectedLabels.keys())
                label_values = list(detectedLabels.values())
                for i in range(len(temp)):
                    tempLabel = label_keys[label_values.index(temp[i])]
                    # Adds the label of the two lowest targets
                    lowerDetects[tempLabel] = i 
                
                if target != "board" and target not in lowerDetects:
                    if "abydos" in target:
                        if "abydos1" in lowerDetects:
                            self.target = "abydos1" 
                        elif "abydos2" in lowerDetects:
                            self.target = "abydos2"
                    elif "earth" in target:
                        if "earth1" in lowerDetects:
                            self.target = "earth1"
                        elif "earth2" in lowerDetects:
                            self.target = "earth2"
                    try:
                        self.targetSide = lowerDetects[self.target]
                    except:
                        print("self.target not in lowerDetects")
                    #print(f"On this side: {self.targetSide}")
            
            # If else, that means we are looking at the board, not at the characters as well, so just get the midpoint of the longest line
            # Since the buoy is a square, the longest line will be the diagonal
            else:
                boardCenter = lines[0].midpoint

        # Get the center of the board -- this will be in form (x, y) = (boardCenter[0], boardCenter[1])
        if boardDetect != None and boardCenter != None:
                # If the discrepancy between boardDetect and the current value of the board's center (based on the ML model) is greater than 
                # 50, make the center of the board the most current value "v". Else just average them to refine the value of the board's center
                boardCenter = [boardDetect[i] if abs(v-boardDetect[i])>50 else v for i,v in enumerate(boardCenter)] # Replacing with "v"
                boardCenter = [(boardCenter[0]+boardDetect[0])/2, (boardCenter[1]+boardDetect[1])/2] # Averaging
        elif boardDetect != None:
            boardCenter = boardDetect
        if boardCenter == None:
            boardCenter = testPoints[0].getPointInt()
        
        boardCenter = [0 if v is None else v for v in boardCenter] # Make sure none of the elements in boardCenter are None. If there is a None value, replace it with a 0
        cv2.circle(frame, (int(boardCenter[0]), int(boardCenter[1])), 5, (255,255,255),-1) # Draw a circle around the center of the board
        
        # If there is no calculated target, get the target that was passed in, else use the calculated target
        if self.target == None:
            targetCenter = detectedLabels.get(target, None)
        else:
            targetCenter = detectedLabels.get(self.target, None)

        if target == "board":
            targetCenter = boardCenter
        if targetCenter!=None:
            if target == "board":
                targetCenter = (int(boardCenter[0]), int(boardCenter[1]))
            else:
                targetCenter = targetCenter.getPointInt()
            cv2.circle(frame, targetCenter, 5, (0,255,0),-1) # Circle the center of the target
        
        toReturn["frame"] = frame
        toReturn["ratio"] = avgOffset
        toReturn["avgDist"] = round(avgDist,3)
        toReturn["side"] = side # Tells us which side of the buoy we are on
        toReturn["targetCenter"] = targetCenter
        toReturn["boardCenter"] = boardCenter
        toReturn["targetSide"] = self.targetSide
        return toReturn

    def yawAndLateralFix(self, center, side):
        """
        Yaw to put the target center in the center of the frame, then move laterally to account for which side of the buoy the 
        sub is on.

        Args:
            center (list): Center of the target (x, y)
            side (str): The side the sub is on (left, right, center)

        Returns:
            tuple: Yaw command, lateral command, state of forward movement (only is True when the sub is centered)
        """
        yaw=0
        lateral=0
        state = False
        # Yaw based on the tolerance and on whether the target center's x-coordinate is to the left or right of the midpoint of the frame
        if(self.step==0):
            xTol = 20
            if(center[0]>self.midX+xTol):
                yaw = 0.7
            elif(center[0]<self.midX-xTol):
                yaw = -0.7
            else:
                self.step=1
                print("switched to 1")

        # Move left or right based on the side the sub is on
        # Positive value = moving right, negative value = moving left
        elif(self.step==1):
            xTol = 250
            #print(center[0])
            if(side=="left"):
                lateral=1.2
            elif(side=="right"):
                lateral=-1.2
            elif(side=="center"):
                state = True
            else:
                #print("can't see all 4")
                pass
            if(center[0]<0+xTol or center[0]>self.midX*2-xTol):
                print("switched to 0")
                self.step=0
        return yaw, lateral, state

    def yawAndLateralMaintain(self, center, ratio):
        """
        Stationkeeping method for mantaining the orientation of the sub

        Args:
            center (list): Target's center (x, y)
            ratio (float): The ratio of the target's x-length/y-length

        Returns:
            tuple: Yaw command, lateral command, state of whether or not to go forward
        """
        # The ratio is x/y so a the smaller the ratio the more askew you are; meaning the more you have to yaw
        # If ratio is close to 1 then you are perfectly aligned
        yaw=0
        lateral=0
        state = False
        tolerance = 20
        if(ratio>self.prevRatio):
            yaw=self.prevYaw*-1
        if center[0]<self.midX-tolerance:
            lateral = -1
        elif center[0]>self.midX+tolerance:
            lateral=1
        else:
            state=True # maybe default too?
        return yaw, lateral, state
    
    def run(self, frame, target, oakd_data):
        """
        Run the buoy CV script

        Args:
            frame: The frame from the camera
            target: Could be anything, for example the character to look for
            oakd_data: This only applies for OAK-D cameras, this is the list of detections from the ML model

        Note (this is the return tuple from calculate_data()):
            toReturn["frame"] = frame
            toReturn["ratio"] = avgOffset
            toReturn["side"] = side
            toReturn["avgDist"] = round(avgDist,3)
            toReturn["targetCenter"] = targetCenter
            toReturn["boardCenter"] = boardCenter

        Returns:
            dictionary, frame: {lateral motion command, forward motion command, yaw motion command, vertical command, side of the buoy we are on, 
            status of the mission (finished or not), state of the mission (end or not)}, visualized frame
        """

        forward = 0
        lateral = 0
        yaw=0
        vertical=0
        end=False
        if oakd_data == None:
            return {}, frame
        # If there's no detection from the ML model, turn to try and get the buoy in the frame
        elif len(oakd_data)==0:
            yaw=1
            return {"yaw": yaw}, frame
        
        #print(target)

        # Calculate the buoy data based on the frame
        result = self.calculate_data(frame, target, oakd_data)
        ratioTol = 1
        ratio = result["ratio"]
        boardCenter = result["boardCenter"]
        dist = result["avgDist"]
        side = result["side"]

        # Put the distance from the buoy and the x/y ratio of the target character on the screen
        if dist!=None:
            cv2.putText(frame, str(dist), (120, 300), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)
        if ratio!=None:
            cv2.putText(frame, str(round(ratio,3)), (120, 200), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        cv2.putText(frame, f"{str('none' if target is None else target)}  {str('none' if target is None else self.target)}", (120, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 120, 0), 2)
        
        #print(ratio, dist)

        # Move forward; fix yaw and heading if the x/y ratio of the target character is off, else maintain the heading and move forward faster to bump the character
        # The ratio is x/y so a the smaller the ratio the more askew you are; meaning the more you have to yaw.
        # If ratio is close to 1 then you are perfectly aligned.
        if dist>2:
            forward = 1
            if(ratio>ratioTol):
                yaw, lateral, do_forward = self.yawAndLateralFix(boardCenter, side)
            else:
                yaw, lateral, do_forward = self.yawAndLateralMaintain(boardCenter, ratio)
                if do_forward:
                    forward=1.3
        elif dist>1:
            targetCenter = result["targetCenter"]
            # If there is no target center, then hit the center of the buoy, this will give at least some points
            if targetCenter==None:
                targetCenter=boardCenter
            yaw, lateral, do_forward = self.yawAndLateralMaintain(targetCenter, ratio)
            if do_forward:
                print("Aligned")
                if dist<1.6:
                    if target=="board":
                        end=True
                    self.finished = True
                    print("Finished")
                forward=0.8

        elif dist==0:
            #print("dist is 0", boardCenter[0])
            tolerance = 20
            if boardCenter[0]<self.midX-tolerance:
                lateral = -1.5
            elif boardCenter[0]>self.midX+tolerance:
                lateral=1.5


        #print("[INFO] Buoy CV run")

        # Store the current yaw and ratio values in the variables that store the "previous" (the ones in the so-called frame before), to prepare 
        # for the next iteration of this function
        self.prevYaw = yaw 
        self.prevRatio = ratio
        
        return {"lateral": lateral, "forward": forward, "yaw": yaw, "vertical": vertical, "targetSide": self.targetSide, "finished": self.finished, "end": end}, result["frame"]


if __name__ == "__main__":
    # This is the code that will be executed if you run this file directly
    # It is here for testing purposes
    # You can run this file independently using: "python -m auv.cv.buoy_cv"

    # Create a CV object with arguments
    cv = CV()

    # Initialize the video object
    cap = cv2.VideoCapture("/Users/eeshvij/Desktop/Code/robosub_2023/testing_data/buoyUSB.mp4")

    while True:
        # Grab a frame
        ret, frame = cap.read()
        if not ret:
            cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            continue

        # Run the CV script
        result = cv.run(frame, "A1", None)

        # Do something with the result
        # print(f"[INFO] {result}")

        # Debug the frame
        cv2.imshow("frame", result[1])
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
        time.sleep(1/40)