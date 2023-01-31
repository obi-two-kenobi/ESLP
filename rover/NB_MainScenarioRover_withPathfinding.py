#NB_MainScenarioRover.py

# Copyright 2020 Jaime Burbano - jaime.burbanovillavicencio@fh-dortmund.de
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#    http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.



#Last Update: 21-10-2020

#Description: Executes the system to get the imiage from the camera, detect the borders
#rectify the ROI (area within the borders), read the scenario image and overlay the 
#obstacles and targets defined there in the real scenario (as a graph).



#Note: This script requires as input the image that describes the scenario. 
#You have to design an image describing the 
#position of the obstacles and the targets, which will be mapped to the image gotten from the camera. 
#The image must be created based on the provided template.
#In the image scenario, rectangles will be interpreted as obstacles and circles as targets and triangles as starting point. 

#ARGUMENTS:
#
# -i / --image  --> the scenario image to upload
# -id           --> the id of the marker on your crawler
# -c / --aws    --> if the application connects or not to aws [True/False] default:False
# -m / --mode   --> the game mode ["bottom"/"top"/"mixed"] default:bottom

#Note: you can exit the program pressing "q" when a window is active.

import numpy as np
import math
import cv2
import time 
import argparse
import transform as tr
import get_xaxis_image_points_opt as marker
import os
import drawScenario
import shapedetector
import configureSystem
from AWSIoTPythonSDK.MQTTLib import AWSIoTMQTTClient
import json

#*** SYSTEM CONFIGURATION
#-----------------------------------------------------------------
myScenarioConfig=configureSystem.configureScenario() #use this object to apply the configurations 
													 #of the system in the configureSystem.py 
							 
myAWSConfig=configureSystem.configureAWS()													 
host = myAWSConfig.get_aws_host()
rootCAPath = myAWSConfig.get_root_file()
certificatePath = myAWSConfig.get_cert_file()
privateKeyPath = myAWSConfig.get_priv_file()
clientId = myAWSConfig.get_thing_name()
topic_rover = myAWSConfig.get_topic("rover")
topic_targer = myAWSConfig.get_topic("target")
useWebsocket=myAWSConfig.useWebsocket


## Class for obstacles
class Obstacle:
    def __init__(self, x:int, y:int, width:int, height:int):
        ## Coordinates
        self.x = x
        self.y = y
        ## Size of obstacle (later refered to as "collision Matrix")
        self.width = width
        self.height = height
        self.sides = []
        ## Genereate vertex matrix (vertexes are the outer points of the obstacle). 
        ## Width and height have to be choosen according to obstacle size (e.g.35cm each)
        ## The navMatrix is used to navigate around the obstacle. The collisionMatrix is used to check for collisions
        ## The navMatrix is the same as the collisionMatrix, but with a safety margin of 1cm
        ## 
        ## 3                    2
        ##  x-----------------x
        ##  |                 |
        ##  |                 | 
        ##  |                 | 
        ##  |                 |
        ##  x-----------------x 
        ## 0                    1

        self.collisionMatrix = [(x-width/2, y-height/2), (x+width/2, y-height/2), (x+width/2, y+height/2), (x-width/2, y+height/2)]
        self.navMatrix = [(x-width/2 - 1, y-height/2 - 1), (x+width/2 +1 , y-height/2 -1), (x+width/2 +1, y+height/2 +1 ), (x-width/2 -1 , y+height/2 +1)]
        
        ## generate side vectors + origins
        for i in range(0,4):
            sideStraight = Straight(PathFinding.getVector(self,self.collisionMatrix[i], self.collisionMatrix[(i+1)%4]),self.collisionMatrix[i], PathFinding.getVectorLength(self, PathFinding.getVector(self,self.collisionMatrix[i], self.collisionMatrix[(i+1)%4])))
            self.sides.append(sideStraight)


class Straight:
    def __init__(self, vector, origin, length):
        self.vector = vector
        self.origin = origin
        self.length = length

class PathFinding:
    ## Returns the two-dimensional vector from start to end
    def getVector(self, start, end):
        return (end[0]-start[0], end[1]-start[1])

    ## Returns the length of a vector
    def getVectorLength(self, vector):
        return math.sqrt(vector[0]**2 + vector[1]**2)

    def checkCaughtWaypoint(self, target, rover):
        ''' Checks if the rover is in the target area. Returns True if it is, False if not'''
        if self.getVectorLength(self.getVector(target, rover)) < 10:
            return True
        else:
            return False

    ## Returns False if obstacles are in the way, True if there is a clear path
    def hasClearPath(self, straight, obstacle:Obstacle):
        ''' Checks if there is a clear path from start to end. Returns False if obstacles are in the way, True if there is a clear path and the intersectionpoint of the path'''
        ## check for intersection with each side of the obstacle
        intersections = []
        for side in obstacle.sides:
            result = self.calculateIntersectionPoint(straight, side)
            if result[0]:
                distanceToIntersection = self.getVectorLength(self.getVector(straight.origin, result[1]))
                intersections.append((distanceToIntersection, result[1]))
        if len(intersections) == 0:
            return True, None
        else:
            return False, min(intersections)[1]

    ## calculate intesection point of the two straights
    def calculateIntersectionPoint(self, straight1:Straight, straight2:Straight):
        ''' Checks if two straights intersect. Returns True if they intersect, False if they don't + the point of intersection'''
        intersectionPoint = None

        ## transform the two straights into a linear equation system and solve it
        a = np.array([[straight1.vector[0], -1*straight2.vector[0]],[straight1.vector[1], -1*straight2.vector[1]]])
        b = np.array([straight2.origin[0]-straight1.origin[0], straight2.origin[1]-straight1.origin[1]])
        intersection = np.linalg.solve(a,b)

        ## check if the intersection point is on both straights
        if(np.allclose(np.dot(a, intersection), b)):
            intersectionPointX = intersection[0] * straight1.vector[0] + straight1.origin[0]
            intersectionPointY = intersection[0] * straight1.vector[1] + straight1.origin[1]
            intersectionPoint = (intersectionPointX, intersectionPointY)
            if intersection is None:
                return False, intersectionPoint
            ## check if the intersection point is in the range of both straights
            if intersection[0] >= 0 and intersection[0] <= straight1.length and intersection[1] >= 0 and intersection[1] <= straight2.length:
                return True, intersectionPoint
            else:
                return False, intersectionPoint
        else:
            return False, intersectionPoint

    ## actual pathfinding method
    def findPath(self, start:tuple, end:tuple, directLine:Straight, obstacles:list):
        ''' Returns a path from start to end. If there is no clear path, the path will be around the obstacles '''
        path = []
        ## check if there is a clear path by iterating through all obstacles
        for obstacle in obstacles:
            if not self.hasClearPath(directLine, obstacle)[0]:
                intersectionPoint = self.hasClearPath(directLine, obstacle)[1]
                print("obstacle in the way, intersection at location:" + str(self.hasClearPath(directLine, obstacle)[1]))
                vertexScores = []
                ## find the optimal vertex to navigate around the obstacle by comparing the distance from the start to the vertex and the distance from the vertex to the end
                for index, vertex in enumerate(obstacle.collisionMatrix):
                    if vertex[0] == intersectionPoint[0] or vertex[1] == intersectionPoint[1]:
                        if self.hasClearPath(Straight(self.getVector(obstacle.navMatrix[index], end),obstacle.navMatrix[index], self.getVectorLength(self.getVector(obstacle.navMatrix[index], end)) ), obstacle)[0]:
                            distanceFromOrigin = self.getVectorLength(self.getVector(start, obstacle.navMatrix[index]))
                            distanceToTarget = self.getVectorLength(self.getVector(obstacle.navMatrix[index], end))
                            vertexScores.append((distanceFromOrigin + distanceToTarget, index))
                bestVertex = min(vertexScores)[1]
                print("optimal vertex to navigate around obstacle is:" + str(obstacle.navMatrix[bestVertex]))

                ## add the optimal vertex to the path
                path.append(obstacle.navMatrix[bestVertex])

                ## update the direct line to the end
                start = obstacle.navMatrix[bestVertex]
                directLine = Straight(self.getVector(start, end), start, self.getVectorLength(self.getVector(start, end)))

        ## after all obstacles have been cleared, add the end to the path
        path.append(end)
        return path									 

def connect_aws():

	if useWebsocket and certificatePath and privateKeyPath:
		print("X.509 cert authentication and WebSocket are mutual exclusive. Please pick one.")
		exit(2)

	if not useWebsocket and (not certificatePath or not privateKeyPath):
		print("Missing credentials for authentication.")
		exit(2)

# Port defaults
	if useWebsocket:  # When no port override for WebSocket, default to 443
		port = 443
	if not useWebsocket:  # When no port override for non-WebSocket, default to 8883
		port = 8883	
	
	
	myAWSIoTMQTTClient = None
	if useWebsocket:
		myAWSIoTMQTTClient = AWSIoTMQTTClient(clientId, useWebsocket=True)
		myAWSIoTMQTTClient.configureEndpoint(host, port)
		myAWSIoTMQTTClient.configureCredentials(rootCAPath)
	else:
		myAWSIoTMQTTClient = AWSIoTMQTTClient(clientId)
		myAWSIoTMQTTClient.configureEndpoint(host, port)
		myAWSIoTMQTTClient.configureCredentials(rootCAPath, privateKeyPath, certificatePath)

	# AWSIoTMQTTClient connection configuration
	myAWSIoTMQTTClient.configureAutoReconnectBackoffTime(1, 32, 20)
	myAWSIoTMQTTClient.configureOfflinePublishQueueing(-1)  # Infinite offline Publish queueing
	myAWSIoTMQTTClient.configureDrainingFrequency(2)  # Draining: 2 Hz
	myAWSIoTMQTTClient.configureConnectDisconnectTimeout(10)  # 10 sec
	myAWSIoTMQTTClient.configureMQTTOperationTimeout(5)  # 5 sec

# Connect and subscribe to AWS IoT
	myAWSIoTMQTTClient.connect()
	return myAWSIoTMQTTClient
					 
													 
def capture_image():
	ret, img = cap.read() #get the frame from the camera
	resize_ratio=1
	#if the received image from the camera is bigger, then resize it
	if max(img.shape) > IMAGE_MAX_WH:
		resize_ratio = float(IMAGE_MAX_WH) / max(img.shape[0], img.shape[1])
		img = cv2.resize(img, (0, 0),fx=resize_ratio,fy=resize_ratio,interpolation=cv2.INTER_AREA)
	return img, resize_ratio

	
if __name__ == "__main__":
	pathfinding = PathFinding()
	#Requires the image of the scenario as input
	ap = argparse.ArgumentParser()
	ap.add_argument("-i", "--image", required=True,
		help="path to the input image")
	ap.add_argument("-c", "--aws", default=False,
		help="connect to aws?")
	ap.add_argument("-id", required=True,
		help="rover id")		
	ap.add_argument("-m", "--mode", default="bottom",
		help="game mode: bottom, top, mixed")		
		
		
	args = vars(ap.parse_args())
	image_scenario = cv2.imread(args["image"])
	connect_to_aws = args["aws"]
	rover_id = args["id"]
	game_mode = args["mode"]	

	
	sd = shapedetector.ShapeDetector() #creates an object to detect the forms existent in the image scenario	
			   
	#Init variables:

	image_scenario, obstacles_scenario, targets_scenario, start_point=sd.get_shapes_coordinates(image_scenario)
	cv2.imshow("Image_scenario", image_scenario)
	size_image_virtual_scenario=tuple(image_scenario.shape[1::-1])
	print("image_virtual_scenario_size: ",size_image_virtual_scenario)	
	print ("detected obstacles in scenario: ", obstacles_scenario)
	print ("detected targets in scenario: ", targets_scenario)
	print ("rover starts in: ", start_point)	

	#::::::::::::::::::::::check aws connection::::::::::::::::
	if connect_to_aws:
		myAWSIoTMQTTClient=connect_aws()
		
	myCameraConfig=configureSystem.configureCamera() #Change to "1" to get the USB camera in configure.System.py

	cap = cv2.VideoCapture(myCameraConfig.get_resource()) #select resource 1 (webcam) --> 0 integrated cam in laptop
	cap.set(cv2.CAP_PROP_FRAME_WIDTH, myCameraConfig.get_camera_width()) #this value could change if using another webcam
	cap.set(cv2.CAP_PROP_FRAME_HEIGHT, myCameraConfig.get_camera_height()) #this value could change if using another webcam
	time.sleep(1) #wait until webcam is ready

	IMAGE_MAX_WH=myScenarioConfig. get_scenario_image_size()#declare the maximum pixels admited (<=1080)
	resize_ratio=1	#initializes the variable

	myBorders=drawScenario.Borders() #creates an object of the class Borders
	myObstacles=drawScenario.Obstacles() #creates an object of the class Obstacles
	myTargets= drawScenario.Targets() #creates an object of the class Targets
	myStart= drawScenario.StartPoint() #creates an object of the class StartPoint	
	border_list_default=[(20,10),(120,10),(120,120),(20,120)]	#default value for the border coordinates (just in case)
	current_target_coordinate=None #Init the target coordinate
	
	myTargets.set_target_list(targets_scenario,game_mode) #formats the target list to determine which one will be shown
	border_list=[] #init the variable where we will put the points of the border markers
	# iterate until we find all the border markers
	while len(border_list)!= len(myScenarioConfig.get_border_ids()):
		border_list.clear() #initializes the variable where we will put the points of the border markers
		img, resize_ratio=capture_image() #gets a frame from the camera
		#size_image_real_scenario=tuple(img.shape[1::-1]) #comment
		#print("image_real_scenario_size: ",size_image_real_scenario) #comment
		my_markers=(marker.get_markers_info(img)) #get markers info (coordinates and angles)			
		#cv2.imshow("Original", img) #shows the original image with the detected markers as a reference
		if my_markers:

			myBorders.draw_borders (img, my_markers, False) #draws the borders 
														    #-->False means we dont use real time border recognition
			borders_coordinates=myBorders.get_borders() #returns the coordinates of the border markers
			#print (borders_coordinates)
			for coordinate_id in borders_coordinates:
				border_list.append(borders_coordinates[coordinate_id]) #appends only the points into a list to be sent to the 
			
			pts = np.array(border_list_default, dtype = "float32") #just in case
			print ("not all borders detected:",my_markers)
			
		time.sleep(0.2)
	
#once we detected the working area, display the scenario and start game	
	continue_game=True
	while(True):
		start=time.time()
		#rectification method
		pts = np.array(border_list, dtype = "float32") #creates an array with the border points gotten before

		img, _=capture_image() #gets a clean capture of the scene (without border markers recognition)
		warped = tr.four_point_transform(img, pts) #cuts the ROI (region of interest) delimited by the
													   #four markers and rectifies it.
		
		size_image_real_scenario=tuple(warped.shape[1::-1]) #gets the size in pixels of the rectified ROI
			
		#Resizes the rectified ROI to form a square equal to the scenario image
		resize_ratio_x=size_image_virtual_scenario[0]/size_image_real_scenario[0] 
		resize_ratio_y=size_image_virtual_scenario[1]/size_image_real_scenario[1] 
		warped = cv2.resize(warped, (0, 0),fx=resize_ratio_x,fy=resize_ratio_y,interpolation=cv2.INTER_AREA)
				
		my_rover_coordinates=(marker.get_markers_info(warped)) #detects the markers in the rectified ROI image 
		myStart.draw_start(warped,start_point[0])	#draw the starting point for the rover	
		list_of_obstacles  = []
		#iterates through the coordinates of the recognized obstacles in the image scenario
		for obstacle_coord in obstacles_scenario:
			warped=myObstacles.draw_obstacles(warped, obstacle_coord) #draws the obstacle from the image scenario
																			#in its equivalent position in the rectified ROI
																			# 
			obsct_obj = Obstacle(obstacle_coord[0], obstacle_coord[1],30,30) #creates an object of the class Obstacle
			list_of_obstacles.append(obsct_obj) #appends the object to a list
		if targets_scenario:
			warped,target_list, current_target_coordinate =myTargets.draw_current_target(warped) #draws the obstacle from the image scenario
			directConnection = Straight(PathFinding.getVector(PathFinding, my_rover_coordinates[int(rover_id)][0], current_target_coordinate), my_rover_coordinates[int(rover_id)][0], PathFinding.getVectorLength(PathFinding, PathFinding.getVector(PathFinding, my_rover_coordinates[int(rover_id)][0], current_target_coordinate)))
			path = pathfinding.findPath(my_rover_coordinates[int(rover_id)][0], current_target_coordinate, directConnection ,list_of_obstacles) #finds the path to the target
			nextWaypoint = path[0]
			if my_rover_coordinates:
				try:
					continue_game=myTargets.check_caught(target_list, my_rover_coordinates[int(rover_id)][0]) 
					continue_path = PathFinding.checkCaughtWaypoint(PathFinding, my_rover_coordinates[int(rover_id)][0], nextWaypoint)
				except:
					print ("no rover_marker in image")														
						
			if continue_path:
				path.pop(0)
			#print("Target:", current_target_coordinate," my rover: ", my_rover_coordinates) #prints {"markerID":[(x coord,y coord), angle]} of the rover in the rectified ROI image
		cv2.imshow("Rectified", warped) #uncomment
		if connect_to_aws:
			mes_rover = json.dumps({'rover': '{}'.format(my_rover_coordinates)})		
			myAWSIoTMQTTClient.publish(topic_rover, mes_rover, 1)	
			mes_target = json.dumps({'target': '{}'.format(current_target_coordinate)})		
			myAWSIoTMQTTClient.publish(topic_targer, mes_target, 1)	
			mes_obs = json.dumps({'obstacles': '{}'.format(obstacles_scenario)})	
			myAWSIoTMQTTClient.publish("esp32/pathFinder", mes_obs, 1)
			
			#print("publishing")
		


		end=time.time()
		#print (end-start )
		time.sleep(0.03)
		
		if cv2.waitKey(1) & 0xFF == ord('q'):
			break
		if not continue_game:
			print ("All targets caught...")
			time.sleep (2)
			break
	cap.release()
	cv2.destroyAllWindows()
	
	
