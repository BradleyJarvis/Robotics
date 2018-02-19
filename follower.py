#!/usr/bin/env python
# BEGIN ALL
import rospy 
import cv2
import numpy
import actionlib
from cv2 import namedWindow
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

#Create Variables
yellow = False
blue = False
green = False
red = False
counter = 0

#Create waypoints for robot to use
waypoints = [
    [(-3.0, 0.0, 0.0), (0.0, 0.0, 0.0, 2.0)], [(1.0, 4.5, 0.0), (0.0, 0.0, 0.0, 1.0)], [(3.0, 1.0, 0.0), (0.0, 0.0, 0.0, 1.0)], [(0.0, -4.5, 0.0), (0.0, 0.0, 0.0, 1.0)], [(-3.0, 2.5, 0.0), (0.0, 0.0, 0.0, 2.0)]
]

class main:
    
  def __init__(self):
    self.bridge = CvBridge()
    #Create window to display image
    namedWindow("Image", 1)
    #Subscribe to LaserScan topic
    self.scan_sub = rospy.Subscriber('turtlebot/scan', 
                                     LaserScan, self.avoid)
    #Subscribe to Image topic                                     
    self.image_sub = rospy.Subscriber('turtlebot/camera/rgb/image_raw', 
                                      Image, self.image_callback)
    #Create publisher to publish twist messages
    self.pub = rospy.Publisher('turtlebot/cmd_vel',
                                       Twist, queue_size=1)
    self.twist = Twist()
    #Create a client to connect to action library
    self.client = actionlib.SimpleActionClient('turtlebot/move_base', MoveBaseAction)
    self.client.wait_for_server()
    
    #Send velocities (2)
    self.send_velocities(2)
    
    
        
  def image_callback(self, data):
    #Convert image to cv2
    image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    #To HSV
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    
    #Global variables
    global green
    global red
    global blue
    global yellow
    global counter
        
    #Search green
    #Set up colour ranges for green
    lower_green = numpy.array([ 40,  100,  100])
    upper_green = numpy.array([60, 255, 255])
    #Create mask with colour ranges
    mask0 = cv2.inRange(hsv, lower_green, upper_green)
    mask0[0:100, 0:640] = 0
    mask0[240:480, 0:640] = 0
    M0 = cv2.moments(mask0)
    
    #Search Red
    #Set up colour ranges for red
    lower_red = numpy.array([ 0,  100,  100])
    upper_red = numpy.array([5, 255, 255])
    #Create mask with colour ranges
    mask1 = cv2.inRange(hsv, lower_red, upper_red)
    mask1[0:100, 0:640] = 0
    mask1[240:480, 0:640] = 0
    M1 = cv2.moments(mask1)
    
    #Search Blue
    #Set up colour ranges for blue
    lower_blue = numpy.array([ 110,  50,  50])
    upper_blue = numpy.array([130, 255, 255])
    #Create mask with colour ranges
    mask2 = cv2.inRange(hsv, lower_blue, upper_blue)
    mask2[0:100, 0:640] = 0
    mask2[240:480, 0:640] = 0
    M2 = cv2.moments(mask2)
    
    #Search Yellow
    #Set up colour ranges for yellow
    lower_yellow = numpy.array([ 30,  100,  100])
    upper_yellow = numpy.array([30, 255, 255])
    #Create mask with colour ranges    
    mask3 = cv2.inRange(hsv, lower_yellow, upper_yellow)
    mask3[0:100, 0:640] = 0
    mask3[240:480, 0:640] = 0
    M3 = cv2.moments(mask3)
    
    
    if green == False:
        #If robot is within certain distance to the coloured object      
        if M0['m00'] > 100000 and M0['m00'] < 4000000:
          #Create red circle over object
          cx = int(M0['m10']/M0['m00'])
          cy = int(M0['m01']/M0['m00'])
          cv2.circle(image, (cx, cy), 20, (0,0,255), -1)
          #Centre of circle
          self.err = cx -240
          #Send velocities(1)
          self.send_velocities(1)
          #If there is a current goal
          if (self.client.get_state() != 2):
              #Cancel goal
              self.client.cancel_goal()
        #If robot is close to the coloured object      
        if M0['m00'] > 4000000:
          #Set green to be found
          green = True
          print 'Found Green'
          #Send velocities(2)
          self.send_velocities(2)
        
    if red == False:
        #If robot is within certain distance to the coloured object
        if M1['m00'] > 100000 and M1['m00'] < 4000000:
          #Create red circle over object
          cx = int(M1['m10']/M1['m00'])
          cy = int(M1['m01']/M1['m00'])
          cv2.circle(image, (cx, cy), 20, (0,0,255), -1)
          #Centre of circle
          self.err = cx -240
          #Send velocities(1)
          self.send_velocities(1)
          #If there is a current goal
          if (self.client.get_state() != 2):
              #Cancel goal
              self.client.cancel_goal()  
        #If robot is close to the coloured object      
        if M1['m00'] > 4000000:
          #Set red to be found
          red = True
          print 'Found Red'
          #Send velocities(2)
          self.send_velocities(2)   
    
    if blue == False:
        #If robot is within certain distance to the coloured object
        if M2['m00'] > 100000 and M2['m00'] < 4000000:
          #Create red circle over object
          cx = int(M2['m10']/M2['m00'])
          cy = int(M2['m01']/M2['m00'])
          cv2.circle(image, (cx, cy), 20, (0,0,255), -1)
          #Centre of circle
          self.err = cx -240
          #Send velocities(1)
          self.send_velocities(1)
          #If there is a current goal
          if (self.client.get_state() != 2):
              #Cancel goal
              self.client.cancel_goal()
        #If robot is close to the coloured object      
        if M2['m00'] > 4000000:
          #Set blue to be found
          blue = True
          print 'Found Blue'
          #Send velocities(2)
          self.send_velocities(2)
        
    if yellow == False:
        #If robot is within certain distance to the coloured object
        if M3['m00'] > 100000 and M3['m00'] < 4000000:
          #Create red circle over object
          cx = int(M3['m10']/M3['m00'])
          cy = int(M3['m01']/M3['m00'])
          cv2.circle(image, (cx, cy), 20, (0,0,255), -1)
          #Centre of circle
          self.err = cx -240
          #Send velocities(1)
          self.send_velocities(1)
          #If there is a current goal
          if (self.client.get_state() != 2):
              #Cancel goal
              self.client.cancel_goal()
        #If robot is close to the coloured object      
        if M3['m00'] > 4000000:
          #Set yellow to be found
          yellow = True
          print 'Found Yellow'
          #Send velocities(2)
          self.send_velocities(2)
    
    #If no colour is in sight or the colour that is in sight has already been found
    if ((M0['m00'] == 0 or green == True) and (M1['m00'] == 0 or red == True) and (M2['m00'] == 0 or blue == True) and (M3['m00'] == 0 or yellow == True) and self.client.get_state() == 2):
        #Send velocities(2)        
        self.send_velocities(2)
    
    #Get the current goal state of the client
    state = self.client.get_state()
    #If the state is complete or rejected
    if (state == 3 or state == 4):
        print 'NEXT'
        #Increase the counter by 1
        counter += 1
        #If the counter is equal to the total amount of waypoints
        if(counter == len(waypoints)):
            #Reset the counter
            counter = 0
            print 'COUNTER RESET'
        #Send velocities(2)    
        self.send_velocities(2)

    #If all colours have been found    
    if (green == True and red == True and blue == True and yellow == True):
        #Send velocities(3)
        self.send_velocities(3)
        print 'Task Complete!'
        #Cancel goal
        if (state != 8):
          self.client.cancel_goal()
    
    #Show the image window with the image
    cv2.imshow("Image", image)
    cv2.waitKey(3) 
    
  def send_velocities(self,x):
      #Global variable    
      global counter
      
      #Create twist message
      twist_msg = Twist()
      
      #If robot is further than 0.8m from an object
      if (self.scan > 0.8):
          #If velocity is 1        
          if (x == 1):
              #Set speed to 0.5            
              twist_msg.linear.x = 0.5
              #Turn to face the centre of the circle that was created over an object
              twist_msg.angular.z = -float(self.err) / 100
          #Else if velocity is 2    
          elif(x == 2):
              #Create a goal with the waypoint from the array of waypoints at the position of the counter            
              goal = self.goal_pose(waypoints[counter])
              #Send the goal to the client
              self.client.send_goal(goal)
              
      #Else if robot is closer than 0.8m to an object    
      elif (self.scan < 0.8):
          #If the object is closer to the left side of the robot than the right side        
          if (self.scanLeft < self.scanRight):
              #Turn right            
              twist_msg.angular.z = 1
              #Set speed to 0.1
              twist_msg.linear.x = 0.1
          #If the object is closer to the right side of the robot than the left side    
          if (self.scanLeft > self.scanRight):
              #Turn left            
              twist_msg.angular.z = -1
              #Set speed to 0.1
              twist_msg.linear.x = 0.1
      #If velocity is 3        
      if(x == 3):
          #Turn right constantly (Spin on spot)        
          twist_msg.angular.z = 2
      
      #Publish the twist message
      self.pub.publish(twist_msg)
      
      
  def avoid(self, data):
      #Use laserScan to scan for objects
      #Set the scan to be the minimum value in the data the laserScan returns (Remove any values that are 'nan')    
      self.scan = min([x for x in data.ranges if str(x) != 'nan'])
      #Set the left side scan to be the minimum value in the left side of the screen (Remove any values that are 'nan')
      self.scanLeft = min([x for x in data.ranges[0:319] if str(x) != 'nan'])
      #Set the right side scan to be the minimum value in the right side of the screen (Remove any values that are 'nan')
      self.scanRight = min([x for x in data.ranges[320:640] if str(x) != 'nan'])


  def goal_pose(self,pose):
      #Create the goal pose    
      goal_pose = MoveBaseGoal()
      #Read in the map to base the goals off of
      goal_pose.target_pose.header.frame_id = 'map'
      #Send X pose
      goal_pose.target_pose.pose.position.x = pose[0][0]
      #Send Y pose
      goal_pose.target_pose.pose.position.y = pose[0][1]
      #Send Z pose
      goal_pose.target_pose.pose.position.z = pose[0][2]
      #Send X orientation
      goal_pose.target_pose.pose.orientation.x = pose[1][0]
      #Send Y orientation
      goal_pose.target_pose.pose.orientation.y = pose[1][1]
      #Send Z orientation
      goal_pose.target_pose.pose.orientation.z = pose[1][2]
      #Send W orientation
      goal_pose.target_pose.pose.orientation.w = pose[1][3]
    
      #Return the goal pose    
      return goal_pose

#Initilise the node
rospy.init_node('main')
follower = main()
rospy.spin()
# END ALL