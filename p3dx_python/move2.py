#!/usr/bin/env python
import rospy # Python library for ROS
from sensor_msgs.msg import LaserScan #import library for lidar sensor
from nav_msgs.msg import Odometry #import library for position and orientation data
from geometry_msgs.msg import Twist,Point
from math import atan2
from tf.transformations import euler_from_quaternion

x=0.0
y=0.0
theta=0.0



class Circling(): #main class
   
    def __init__(self): #main function

	global circle

        circle = Twist() #create object of twist type  
        self.pub = rospy.Publisher("/p3dx/cmd_vel", Twist, queue_size=10) #publish message
        self.sub = rospy.Subscriber("/p3dx/laser/scan", LaserScan, self.callback) #subscribe message 
        self.sub = rospy.Subscriber("/p3dx/odom", Odometry, self.odometry) #subscribe message


    def odometry(self,msg): #function for odometry
        print msg.pose.pose #print position and orientation of turtlebot

        global x
        global y
        global z
        global w



        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        self.rx = msg.pose.pose.orientation.x
        self.ry = msg.pose.pose.orientation.y
        self.rz = msg.pose.pose.orientation.z
        self.rw = msg.pose.pose.orientation.w

        

    def callback(self, msg): #function for obstacle avoidance
        global roll
        global pitch
        global theta
        global goal

        print '-------RECEIVING LIDAR SENSOR DATA-------'
        print 'Front: {}'.format(msg.ranges[0]) #lidar data for front side
        print 'Left:  {}'.format(msg.ranges[90]) #lidar data for left side
        print 'Right: {}'.format(msg.ranges[270]) #lidar data for right side
        print 'Back: {}'.format(msg.ranges[180]) #lidar data for back side
        goal=Point()
        goal.x=5
        goal.y=5
        (roll, pitch, theta) = euler_from_quaternion([self.rx, self.ry, self.rz, self.rw])

        inc_x = goal.x -x
        inc_y = goal.y -y
        angle_to_goal = atan2(inc_y, inc_x)

        print('roll:',roll,'pitch:',pitch,'theta:',theta)
        print('inc_x:',inc_x,'inc_y:',inc_y)
        print('angle_to_goal:',angle_to_goal)
      	
      	#Obstacle Avoidance
        self.distance = 0.7
        if msg.ranges[0] > self.distance and msg.ranges[15] > self.distance and msg.ranges[345] > self.distance: 
        #when no any obstacle near detected
    	    if abs(angle_to_goal - theta) > 0.1:  #can't change the dircetion it makes only one direction...
       		circle.linear.x = 0.0
        	circle.angular.z = 0.3
    	    else:
        	circle.linear.x = 0.5
        	circle.angular.z = 0.0
            rospy.loginfo("Circling") #state situation constantly

        else: #when an obstacle near detected
            rospy.loginfo("An Obstacle Near Detected") #state case of detection
            circle.linear.x = 0.0 # stop
            circle.angular.z = 0.5 # rotate counter-clockwise

            if msg.ranges[0] > self.distance and msg.ranges[15] > self.distance and msg.ranges[345] > self.distance and msg.ranges[45] > self.distance and msg.ranges[315] > self.distance:
                #when no any obstacle near detected after rotation
                if abs(angle_to_goal - theta) > 0.1:
                    circle.linear.x = 0.0
                    circle.angular.z = 0.3
                else:
                    circle.linear.x = 0.5
                    circle.angular.z = 0.0

	self.pub.publish(circle) # publish the move object
    



if __name__ == '__main__':

    rospy.init_node('obstacle_avoidance_node') #initilize node
 
    Circling() #run class 
    rospy.spin() #loop it

