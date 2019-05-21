#!/usr/bin/python
import numpy as np
import rospy
from tf.transformations import euler_from_quaternion
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Float64
from sensor_msgs.msg import LaserScan
import hybrid_automata as HA

pub_topic_l = '/omni_bot/left_wheel_controller/command'
pub_topic_r = '/omni_bot/right_wheel_controller/command'
pub_topic_f = '/omni_bot/front_wheel_controller/command'
sub_topic = '/gazebo/model_states'
sub_topic_laser = '/omni_bot/laser/scan'
R = HA.R
FREQUENCY = 100 #Hz																# Frequency of ROS publishing commands
T_END = 10 #secs																# End time for following a curve as a function of time
currPosition = HA.Position(0,0,0)
desiredPosition = HA.Position(5,5,0)											# Set desired coordinate in xy plane here
desiredHeading = 0
laser_data = HA.laserSensor([],[],0)

def curve(t):																	# Derivate of curve to be followed w.r.t. time
	y = np.sin(2*3.14*t/5)
	return y
def curve_derivative(t):
	y_derivative = (2*3.14/5)*np.cos(2*3.14*t/5)
	return y_derivative
def readOdom(msg):																# Position and Heading Feedback
	global currPosition
	currPosition.x = msg.pose[1].position.x
	currPosition.y = msg.pose[1].position.y
	rot = msg.pose[1].orientation
	(_,_,currPosition.yaw) = euler_from_quaternion([rot.x,rot.y,rot.z,rot.w])
def readLaser(msg):																# Obstacale Heading Feedback
	global laser_data
	data = msg.ranges
	mag = []
	angle = []
	for i in range(len(data)):
		if data[i] != float('inf') and data[i] != -float('inf'):
			angle.append(i*6.28/720-3.14)
			mag.append(data[i])
	laser_data.mag = mag
	laser_data.angle = angle
	laser_data.size = len(angle)

controller = rospy.init_node('G2G_Controller',anonymous=True)					# Initialize ROS node
sub = rospy.Subscriber(sub_topic,ModelStates,readOdom)							# Initialize position information subscriber
sub_laser = rospy.Subscriber(sub_topic_laser,LaserScan,readLaser)				# Initialize laser scanner subscriber
pub_l = rospy.Publisher(pub_topic_l,Float64,queue_size=10)						# Initialize left wheel RPM publisher
pub_r = rospy.Publisher(pub_topic_r,Float64,queue_size=10)						# Initialize right wheel RPM publisher
pub_f = rospy.Publisher(pub_topic_f,Float64,queue_size=10)						# Initialize front wheel RPM publisher
r = rospy.Rate(FREQUENCY)
endPosition = HA.Position(5,0)
while not rospy.is_shutdown():
	y = curve(currPosition.x)
#	(robot,rpmLeft,rpmRight,rpmFront) = HA.GTG(currPosition,desiredPosition,desiredHeading)
	(robot,rpmLeft,rpmRight,rpmFront) = HA.followCurve(currPosition,endPosition,y,np.arctan2(curve_derivative(currPosition.x),1))
	pub_l.publish(rpmLeft)
	pub_r.publish(rpmRight)
	pub_f.publish(-rpmFront)	#minus sign because the wheel was joined in URDF design flipped from normal position
#	print((currPosition.x,currPosition.y,np.rad2deg(currPosition.yaw)))
	r.sleep()
