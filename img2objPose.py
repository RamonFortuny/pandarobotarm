#!/usr/bin/env python
import rospy
import cv2
import numpy as np
import imutils
import argparse
from math import atan2, cos, sin, sqrt, pi
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import String, Float64
from scipy.spatial import distance as dist
from imutils import perspective
from imutils import contours
from geometry_msgs.msg import Twist 
from gaussian import *
import math
from imutils import perspective
from scipy.spatial import distance as dist
from math import atan2, cos, sin, sqrt, pi
from collections import defaultdict
import time

class Image2Object(object):
    def __init__(self):
        # Params
        self.br = CvBridge()
        self.loop_rate = rospy.Rate(10)
        self.printed = False
        self.calculateBestPoints = False
        
        #Object detector
        self.image = None
        self.image_message = None
        self.objsToGrabTransformedSend = []
        self.iterations = 0
        self.prevObjVisible =0
        self.AllBestPoints = []
        self.BestPointsCounter = []
        self.prevAllBestPoints = []

        #Messenger
        self.action = ""
        self.msg2Controller = ""
        self.awaitingResponse = True

        #Coordinates Msg
        self.msg = Twist()

        # Publishers
        self.pub = rospy.Publisher('/image_raw_but_better', Image,queue_size=10)
        self.pubM = rospy.Publisher('/camera2controller', String, queue_size=10)
        self.PubCoor = rospy.Publisher('/objCoordinates', Twist, queue_size=10)
        #self.PubGa = rospy.Publisher('/camera2gaussian', Float64, queue_size=10)

        # Subscribers
        rospy.Subscriber("/image_raw",Image,self.callback,queue_size=1)
        rospy.Subscriber("/controller2camera",String, self.callbackMsg,queue_size=10)

        # Gaussian
        self.pWidth = 0.0
        
    def callbackMsg(self, data):
        self.action = data.data

    '''
    Mission Plan for picking up objects at unknown locations
    '''
    def switch(self, lang):
        if lang == "Are you ready?":
            self.msg2Controller = "Ready!"
        else:
            #print("lang:" ,lang)
            #print("self.objsToGrabTransformedSend",self.objsToGrabTransformedSend)
            if lang == "Awaiting coordinates":
                if len(self.objsToGrabTransformedSend) > 0:
                    self.printed = False
                    self.calculateBestPoints = False
                    py = self.objsToGrabTransformedSend[0][0]
                    px = self.objsToGrabTransformedSend[0][1]
                    pz = self.objsToGrabTransformedSend[0][2]
                    oz = self.objsToGrabTransformedSend[0][4]
                    self.pWidth = self.objsToGrabTransformedSend[0][5]
                    
                    self.pubCoordinates(py, px, pz, 0.0, 0.0, oz, 0.0)
                    #self.pubCoordinates(0.0, 0.0, pz, 0.0, 0.0, 1.57, 0.0)
                    self.msg2Controller = "Coordinates are being published..."
                else:
                    self.calculateBestPoints = True  # start calculating best points
                    self.msg2Controller = "Ready!"
                    
            elif lang == "Reset":
                self.pWidth = 0.0
                self.objsToGrabTransformedSend=[]
                self.msg2Controller = "Ready!"

            # elif lang == "Good job gripper... We did it!":
            #     self.msg2Controller = "Likewise!"    
            # elif (len(self.objsToGrabTransformedSend) < 1):
            #     self.msg2Controller = "No objects visible"

        self.pubM.publish(self.msg2Controller)

    '''
    Publish Coordinates
    '''
    def pubCoordinates(self, py, px, pz, ox, oy, oz, ow):
        self.msg.linear.y = py
        self.msg.linear.x = px
        self.msg.linear.z = pz
        self.msg.angular.x = 0.0
        self.msg.angular.y = 0.0
        self.msg.angular.z = oz
        self.PubCoor.publish(self.msg)

    def slope_at_contour_points(self, contour, span): #contour: [[[x1,y1]], [[x2,y2]], ...]
        # Calculate the number of points in the contour
        n_points = len(contour)
        #print(n_points)
        # Initialize an empty list to store the slopes
        slopes = []

        half_span = span // 2  # Floor division to get an integer

        for i in range(n_points):
            # Get the point 'half_span' steps before and 'half_span' steps after
            if i - half_span >= 0:
                previous_point = contour[i - half_span][0]
            else:
                # Wrap around to the end of the contour if necessary
                previous_point = contour[i - half_span + n_points][0]

            if i + half_span < n_points:
                next_point = contour[i + half_span][0]
            else:
                # Wrap around to the start of the contour if necessary
                next_point = contour[i + half_span - n_points][0]

            # Calculate the difference in x and y coordinates
            dx = next_point[0] - previous_point[0]
            dy = next_point[1] - previous_point[1]


            slope_rad = round(math.atan2(dy, dx)-pi,2) #values go from 0 to 2pi

            slopes.append(-slope_rad)
            
        #print(slopes)
            
        return slopes

    def draw_slope_lines(self, image, contour, slopes, line_length=100):  #contour: [[[x1,y1]], [[x2,y2]], ...]
        half_length = line_length // 2
        quarter_length = int(len(contour) * 1)  # x % of the contour length

        for i, (point, slope) in enumerate(zip(contour, slopes)):
            if i % 6 == 0 and i<quarter_length:  # This condition ensures we only draw every fourth line
                x, y = point[0]
                start_x = int(x - half_length * np.cos(-1*slope))
                start_y = int(y - half_length * np.sin(-1*slope))
                end_x = int(x + half_length * np.cos(-1*slope))
                end_y = int(y + half_length * np.sin(-1*slope))
                cv2.line(image, (start_x, start_y), (end_x, end_y), (255, 255, 255), 1)
            #print(point,slope)

    def get_best_point_pair(self, contour, slopes, weights):  #contour: [[[x1,y1]], [[x2,y2]], ...]
        slopes = np.array(slopes)
        contour = np.squeeze(contour)

        max_score = -float('inf')  # initialize maximum score
        best_pair = None  # initialize best pair

        contour_center = np.mean(contour, axis=0)

        max_dist_to_center = np.max(np.linalg.norm(contour - contour_center, axis=1))

        max_point = np.max(contour, axis=0)
        min_point = np.min(contour, axis=0)
        max_dist_between_points = np.linalg.norm(max_point - min_point)
        # max_dist_between_points = 100
        #print(max_point,min_point,max_dist_between_points)

        # Group points by y-coordinate
        grouped = defaultdict(list)
        for idx, point in enumerate(contour):
            grouped[point[1]].append((point, slopes[idx]))

        for y, points in grouped.items():
            points.sort(key=lambda p: p[0][0]) # Sort points by x-coordinate
            #print(points) #Looks like:
                # [(array([1163,  999], dtype=int32), -5.0), (array([1223,  999], dtype=int32), 5.0)]
                # [(array([1163, 1000], dtype=int32), -5.0), (array([1223, 1000], dtype=int32), 5.0)]
                # [(array([1163, 1001], dtype=int32), -10.0), (array([1223, 1001], dtype=int32), 5.0)]
                # [(array([1163, 1002], dtype=int32), -10.0), (array([1223, 1002], dtype=int32), 10.0)]
                # [(array([1163, 1003], dtype=int32), -10.0), (array([1223, 1003], dtype=int32), 10.0)]
            
            if len(points) > 15:  # If we have more than 15 points in the same line, continue
                continue
            #print(points[1][0][0]) #num_column #0 #0=x,1=y

            # Look for the first point and then next one which is at least 10 pixels further in the x-direction
            i = 0
            while i < len(points):
                gap_found = False
                for j in range(i + 1, len(points)):
                    if 10<points[j][0][0] - points[i][0][0]<100 :  # This checks if there's a gap
                        # We found a gap, let's check the pair of points at the ends of the gap
                        point1, slope1 = points[i]
                        point2, slope2 = points[j]

                        slope_diff = abs(slope1 - slope2)  # between 0 and 2pi
                        slope_score = round(1 - abs(np.pi - slope_diff) / np.pi, 4)  # the closer to pi (the slope_diff) the better
                        #print(point1,point2,slope1,slope2,slope_score)

                        midpoint = np.mean([point1, point2], axis=0)

                        dist_to_center = np.linalg.norm(midpoint - contour_center)
                        dist_to_center_score = round(1 - (dist_to_center / max_dist_to_center),4)  # normalized score
                        #print(point1,point2,midpoint,dist_to_center,dist_to_center_score)

                        point_dist = np.linalg.norm(point1 - point2)
                        point_dist_score = round(1 - (point_dist / max_dist_between_points),4)  # normalized score
                        #print(point1,point2,point_dist, point_dist_score)

                        score = round(weights['slope'] * slope_score + \
                                weights['center_dist'] * dist_to_center_score + \
                                weights['point_dist'] * point_dist_score,4)
                        #print(point1,point2,slope_score,dist_to_center_score,point_dist_score,score)


                        if score > max_score:
                            max_score = score
                            best_pair = [point1, point2]

                        gap_found = True
                        break
                if gap_found:
                    i = j
                else:
                    i += 1   
        #print(best_pair)   
        return best_pair, max_score
    
    def get_best_point_pair_for_all_rotations(self, contour, slopes, weights, image_shape):
        rotation_angles = np.linspace(0, 180, 13)  # 8 angles from 0 to 180 degrees (0 , 25.7, 51.4, ... 180)
        best_pair_overall = None
        best_score_overall = -float('inf')
        best_rotation_angle_overall = 0

        center = np.array([image_shape[1] / 2, image_shape[0] / 2])  # Center of the image (width/2, height/2)
        #print(rotation_angles)

        for rotation_angle in rotation_angles[:-1]: #for all angles in rotation_angles except the last one (180, because it's same as 0)
            # Rotate points
            rotated_contour = self.rotate_contour_points(contour, rotation_angle, center)
            #print(rotated_contour)
            # Call get_best_point_pair function
            best_pair, best_score = self.get_best_point_pair(rotated_contour, slopes, weights)
            #print(self.rotate_best_pair_points(np.array(best_pair), -rotation_angle, center),best_score,rotation_angle)
            if best_score > best_score_overall:
                best_score_overall = best_score
                #print("best pair", best_pair, "rotation angle", rotation_angle)
                best_pair_overall = self.rotate_best_pair_points(np.array(best_pair), -rotation_angle, center)
                #print("BEST OVERALL", best_pair_overall)
                best_rotation_angle_overall = rotation_angle

        #print(best_pair_overall,best_score_overall,best_rotation_angle_overall)
        return best_pair_overall, best_score_overall, best_rotation_angle_overall

    
    def get_best_point_pair_two_contours(self, outer_contour, inner_contour, slopes_outer, slopes_inner, weights):
        outer_contour = np.squeeze(outer_contour)
        inner_contour = np.squeeze(inner_contour)

        max_score = -float('inf')  # initialize maximum score
        best_pair = None  # initialize best pair


        contour_center=np.mean(inner_contour, axis=0)
        max_dist_to_center = np.max(np.linalg.norm(outer_contour - contour_center, axis=1))

        max_point = np.max(outer_contour, axis=0)
        min_point = np.max(inner_contour, axis=0)
        max_dist_between_points = np.linalg.norm(max_point - min_point)

        # Group points by y-coordinate
        outer_grouped = defaultdict(list)
        inner_grouped = defaultdict(list)

        for idx, point in enumerate(outer_contour):
            outer_grouped[point[1]].append((point, slopes_outer[idx]))
        for idx, point in enumerate(inner_contour):
            inner_grouped[point[1]].append((point, slopes_inner[idx]))  

        
        # For each y-coordinate, compare points with the same y-coordinate
        for y, outer_points in outer_grouped.items():
            inner_points = inner_grouped.get(y, [])
            

            # If we have more than 15 points in the same line, continue
            if len(outer_points) > 15 or len(inner_points) > 15:
                continue
            
            #print("outer", outer_points, "inner", inner_points)
            # For each pair of points with the same y-coordinate, find the best pair
            for outer_point, outer_slope in outer_points:
                for inner_point, inner_slope in inner_points:
                    slope_diff = abs(outer_slope - inner_slope)  # between 0 and 2pi
                    slope_score = round(1 - abs(np.pi - slope_diff) / np.pi, 4)  # the closer to pi the better

                    midpoint = np.mean([outer_point, inner_point], axis=0)

                    dist_to_center = np.linalg.norm(midpoint - contour_center)
                    dist_to_center_score = round(1 - (dist_to_center / max_dist_to_center),4)  # normalized score

                    point_dist = np.linalg.norm(outer_point - inner_point)
                    point_dist_score = max(round(1 - (point_dist / max_dist_between_points),4),0)  # normalized score

                    score = round(weights['slope'] * slope_score + \
                            weights['center_dist'] * dist_to_center_score + \
                            weights['point_dist'] * point_dist_score,4)

                    if score > max_score:
                        max_score = score
                        best_pair = [outer_point, inner_point]
        #print(best_pair)
        return best_pair, max_score                         

    def get_best_point_pair_for_all_rotations_two_contours(self, outer_contour, inner_contour, slopes_outer, slopes_inner, weights, image_shape):
        rotation_angles = np.linspace(0, 180, 13)  # 8 angles from 0 to 180 degrees (0 , 25.7, 51.4, ... 180)
        best_pair_overall = None
        best_score_overall = -float('inf')
        best_rotation_angle_overall = 0

        center = np.array([image_shape[1] / 2, image_shape[0] / 2])  # Center of the image (width/2, height/2)
        #print("rotation angles",rotation_angles)

        for rotation_angle in rotation_angles[:-1]: #for all angles in rotation_angles except the last one (180, because it's same as 0)
            # Rotate points
            #print("Checking angle:",rotation_angle)
            rotated_outer_contour = self.rotate_contour_points(outer_contour, rotation_angle, center)
            rotated_inner_contour = self.rotate_contour_points(inner_contour, rotation_angle, center)
            #print(rotation_angle, "outer contour:", outer_contour, "inner contour:", inner_contour)
            # Call get_best_point_pair function
            best_pair, best_score = self.get_best_point_pair_two_contours(rotated_outer_contour, rotated_inner_contour, slopes_outer, slopes_inner, weights)
            #print(self.rotate_best_pair_points(np.array(best_pair), -rotation_angle, center),best_score,rotation_angle)
            if best_score > best_score_overall:
                best_score_overall = best_score
                #print("best pair", best_pair, "rotation angle", rotation_angle)
                best_pair_overall = self.rotate_best_pair_points(np.array(best_pair), -rotation_angle, center)
                #print("BEST OVERALL", best_pair_overall)
                best_rotation_angle_overall = rotation_angle

        #print(best_pair_overall,best_score_overall,best_rotation_angle_overall)
        return best_pair_overall, best_score_overall, best_rotation_angle_overall

    def rotate_contour_points(self, points, angle, center):
        if len(points) == 0:
            return []
        rotation_matrix = cv2.getRotationMatrix2D(tuple(center), angle, 1).T
        points_rotated = np.dot(points - center, rotation_matrix[:2, :2]) + center

        # Round to nearest integer and convert to integer type, then remove extra dimension.
        points_rotated = np.round(points_rotated).astype(int)
        #print(points_rotated)
        return points_rotated
 

    def rotate_best_pair_points(self, points, angle, center):
        if len(points) == 0:
            print("len of points 0")
            return []
        #print("points" , points)
        rotation_matrix = cv2.getRotationMatrix2D(tuple(center), angle, 1).T
        points_rotated = np.dot(points.reshape(-1, 2) - center, rotation_matrix[:2, :2]) + center
        points_rotated=np.round(points_rotated).astype(int)
        #print("points_rotated", points_rotated)
        return [np.array(point) for point in points_rotated]
 




    def draw_best_points(self, image, best_pair):

        # Draw the points in blue
        #print(best_pair,best_pair[0])
        cv2.circle(image, tuple(best_pair[0].astype(int)), 4, (255, 0, 0), -1)  # Circle radius is set to 4, change it if needed
        cv2.circle(image, tuple(best_pair[1].astype(int)), 4, (255, 0, 0), -1)
        
        # Calculate the midpoint
        midpoint = np.mean([best_pair[0], best_pair[1]], axis=0).astype(int)

        
        # Draw the midpoint in red
        cv2.circle(image, tuple(midpoint), 4, (0, 0, 255), -1)
        
        return image



    '''
    Object detector that creates an array with the tranformed x and y coordinates (https://automaticaddison.com/how-to-determine-the-orientation-of-an-object-using-opencv/)
    '''
    def callback(self, msg):
        #Intialize
        objsToGrabTransformed = []
        BestPointOverall=None
        self.image = self.br.imgmsg_to_cv2(msg)
        oz = 0.0
        objsToGrab=[]

        if not self.calculateBestPoints:
            #self.image_message = self.br.cv2_to_imgmsg(self.image, encoding="passthrough")
            return

        # Get image dimensions 1544, 2064
        image_shape = self.image.shape[:2]
        (h, w) = self.image.shape[:2]
        (cX, cY) = (w // 2, h // 2)
        center= np.array([w / 2, h / 2])

        # Rotate Image by 1 degree
        M = cv2.getRotationMatrix2D((cX, cY), -1, 1.0)
        rotated = cv2.warpAffine(self.image, M, (w, h))

        # Add black rectangles everywhere except for the converyor belt
        cv2.rectangle(rotated, (0,0), (2064, 930), (0,0,0), -1)
        cv2.rectangle(rotated, (0,0), (550, 1544), (0,0,0), -1)
        cv2.rectangle(rotated, (0,1095), (2064, 1544), (0,0,0), -1)
        cv2.rectangle(rotated, (1780,0), (2064, 1544), (0,0,0), -1)

        # Gaussain Blur to remove noise
        blurred = cv2.GaussianBlur(rotated, (7, 7), 0)

        # Thresholding
        ret,imgt = cv2.threshold(blurred,29,255,cv2.THRESH_BINARY) 

        # Remove more noise using erosion and dialation
        kernel = np.ones((5, 5), np.uint8)
        img_erosion = cv2.erode(imgt, kernel, iterations=2)
        img_dilation = cv2.dilate(img_erosion, kernel, iterations=2)

        # Find contours of the found objects and grab them
        cnts = cv2.findContours(img_dilation, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
        contours = cnts[0]  #R Save list of contours (Contour1(all [x,y]), Contour2..)
        #contours[0] # It will be the first contour in the list
        #contours[0][0] It will be the first point in the first contour [[x,y]]
        #contours[0][0][0] It will be the first point in the first contour[x,y]
        #contours[0][1][0][0] #x of the second point in the first contour [x]
        #contours[0][1][0][1] #y of the second point in the first contour [y]
        
        # Filtering contours based on number of points
        min_points = 100
        filtered_contours = [contour for contour in contours if len(contour) >= min_points]
        #print(filtered_contours)

        objects = []  # Initialize an empty list to store objects
        centers = []  # Initialize an empty list to store centers of objects

        for i, contour1 in enumerate(filtered_contours):
            center1 = np.mean(contour1, axis=0)  # Calculate center of the contour
            found_object = None
            found_index = None

            # Try to find an existing object that this contour belongs to
            for index, obj in enumerate(objects):
                # Check only the first contour in the object
                contour2 = obj[0]
                if cv2.pointPolygonTest(contour2, tuple(center1[0]), False) > 0:
                    found_object = obj
                    found_index = index
                    break

            if found_object is None:
                # If this contour doesn't belong to any existing object, create a new one
                objects.append([contour1])
                centers.append(center1)
            else:
                # Otherwise, add it to the found object
                found_object.append(contour1)
                # Update the center of the object
                centers[found_index] = np.mean([centers[found_index], center1], axis=0)

        # for index, (obj, center) in enumerate(zip(objects, centers)):
        #     #print(f"Object {index + 1}'s center: {center}")
        #     #print(f"Object {index + 1} has {len(obj)} contour(s)")
        
        # Find the object with the smallest x-coordinate for its center
        if not centers:
            #print("No centers detected!")
            self.AllBestPoints = []
            self.BestPointsCounter = []
            self.image_message = self.br.cv2_to_imgmsg(self.image, encoding="passthrough")
            return
        min_x_center_index = np.argmin([center[0][0] for center in centers])  # Find the index of the center with smallest x-coordinate
        selected_object = objects[min_x_center_index]  # Use the index to get the corresponding object
        selected_center = centers[min_x_center_index]  # And the corresponding center

        #print(f"Selected object's center: {selected_center}")
        #print(f"Selected object has {len(selected_object)} contour(s)")

        best_pair=[]
        if len(selected_object)==1:
            #print(selected_object)
            first_contour = selected_object[0]
            first_contour = self.rotate_contour_points(first_contour, 1, center)  # rotate by 1 degree, compensation of the rotation done with the image filtering/processing steps
            cv2.drawContours(self.image, first_contour, -1, (255, 255, 255), 2)  # draw contour
            slopes = self.slope_at_contour_points(first_contour, 10)
            self.draw_slope_lines(self.image, first_contour, slopes)
            weights = {'slope': 0.5, 'center_dist': 0.3, 'point_dist': 0.2}
            best_pair, best_score, best_rotation = self.get_best_point_pair_for_all_rotations(first_contour, slopes, weights, image_shape)
            self.draw_best_points(self.image, best_pair)
            midpoint = np.mean([best_pair[0], best_pair[1]], axis=0).astype(int)
            oz = round(np.deg2rad(best_rotation), 2)
            w = 0  # to be removed
            objsToGrab=(midpoint[0], midpoint[1], 0, 0, oz, w)
        else:
            all_slopes=[]
            
            for i in range(len(selected_object)):
                selected_object[i] = self.rotate_contour_points(selected_object[i], 1, center) #we rotate by 1 degree to hape the exact position of the contour on the original image
                cv2.drawContours(self.image, selected_object[i], -1, (255, 255, 255), 2) #draws exact points found by contour function
                slopes = self.slope_at_contour_points(selected_object[i],10)
                all_slopes.append(slopes)
                self.draw_slope_lines(self.image, selected_object[i], slopes)
            weights = {'slope': 0.4, 'center_dist': 0.4, 'point_dist': 0.2}
            #print(filtered_contours[1])
            #print("ABOUT TO GET BEST POINT PAIR")
            best_pair, best_score, best_rotation = self.get_best_point_pair_for_all_rotations_two_contours(selected_object[0], selected_object[1],all_slopes[0],all_slopes[1], weights, image_shape)
            self.draw_best_points(self.image, best_pair)
            #print(best_pair, best_rotation)
            midpoint = np.mean([best_pair[0], best_pair[1]], axis=0).astype(int)
            oz = round(np.deg2rad(best_rotation), 2)
            w=0 #to be removed
            objsToGrab=(midpoint[0], midpoint[1], 0, 0, oz, w)

        # Check if objsToGrab is in self.AllBestPoints
        if not any(np.array_equal(objsToGrab, existing) for existing in self.AllBestPoints):
            # If not, add it to the list and initialize a counter for it
            self.AllBestPoints.append(objsToGrab)
            self.BestPointsCounter.append(1)
        else:
            # If it is, increment the counter for it
            for i, existing in enumerate(self.AllBestPoints):
                if np.array_equal(objsToGrab, existing):
                    self.BestPointsCounter[i] += 1
                    if self.BestPointsCounter[i] == 15:
                        # Store the value
                        BestPointOverall = self.AllBestPoints[i]
                        # Reset all to the starting point
                        self.AllBestPoints = []
                        self.BestPointsCounter = []
                        
        #self.prevAllBestPoints = self.AllBestPoints.copy()
        #print(self.AllBestPoints, "counter:", self.BestPointsCounter)
        if BestPointOverall is not None:
            xPixel = BestPointOverall[0] - 556
            yPixel = BestPointOverall[1] - 935
            
            xMeter = xPixel * (1.416/1195)
            yMeter = yPixel * (0.192/166)
            zMeter = BestPointOverall[2]
            oz = BestPointOverall[4]
            wi = BestPointOverall[5]*(1448/1230)
            objsToGrabTransformed.append((yMeter, xMeter, zMeter, 0, oz, wi))
            #print("BestPointOverall: ",BestPointOverall)
            #print("objsToGrabTransformed: ",objsToGrabTransformed)
            self.objsToGrabTransformedSend = objsToGrabTransformed 
        #print("Best point overall:",BestPointOverall)


        self.image_message = self.br.cv2_to_imgmsg(self.image, encoding="passthrough")

    def start(self):
        rospy.loginfo("Timing images")
        while not rospy.is_shutdown():
            self.switch(self.action)
            if self.image_message is not None:
                self.pub.publish(self.image_message)
                #self.PubGa.publish(self.pWidth)

            self.loop_rate.sleep()

if __name__ == '__main__':
    rospy.init_node("img2objPose", anonymous=True)
    my_node = Image2Object()
    my_node.start()