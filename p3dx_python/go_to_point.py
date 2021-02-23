import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from math import atan2
import math

x = 0.0
y = 0.0 
theta = 0.0

def newOdom(msg):
    global x
    global y
    global theta

    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y

    rot_q = msg.pose.pose.orientation
    (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
    print('theta good?', theta)

rospy.init_node("speed_controller")

sub = rospy.Subscriber("/p3dx/odom", Odometry, newOdom)
pub = rospy.Publisher("/p3dx/cmd_vel", Twist, queue_size = 10)

speed = Twist()

r = rospy.Rate(4)

goal = Point()
goal.x = 5
goal.y = 5

while not rospy.is_shutdown():
    inc_x = goal.x -x
    inc_y = goal.y -y


    angle_to_goal = atan2(inc_y, inc_x)
    print('angle_to_goal:',angle_to_goal,'inc_y:',inc_y,'inc_x:',inc_x)

    if abs(angle_to_goal - theta) > 0.1:
	print('abs angle_to_goal:', abs(angle_to_goal-theta))
        speed.linear.x = 0.0
        speed.angular.z = 0.3
    else:
	print('abs angle_to_goal:', abs(angle_to_goal-theta))
        speed.linear.x = 0.5
        speed.angular.z = 0.0


    pub.publish(speed)
    r.sleep()    



  
