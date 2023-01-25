#!/usr/bin/env python3

'''
This python file runs a ROS-node of name drone_control which holds the position of e-Drone on the given dummy.
This node publishes and subsribes the following topics:
		PUBLICATIONS			SUBSCRIPTIONS
		/drone_command			/whycon/poses
		/alt_error				/pid_tuning_altitude
		/pitch_error			/pid_tuning_pitch
		/roll_error				/pid_tuning_roll
					
								
Rather than using different variables, use list. eg : self.setpoint = [1,2,3], where index corresponds to x,y,z ...rather than defining self.x_setpoint = 1, self.y_setpoint = 2
CODE MODULARITY AND TECHNIQUES MENTIONED LIKE THIS WILL HELP YOU GAINING MORE MARKS WHILE CODE EVALUATION.	
'''

# Importing the required libraries

from edrone_client.msg import *
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Int16
from std_msgs.msg import Int64
from std_msgs.msg import Float64
from pid_tune.msg import PidTune
import rospy
import time


class Edrone():
	"""docstring for Edrone"""
	def __init__(self):
		
		rospy.init_node('drone_control')	# initializing ros node with name drone_control

		# This corresponds to your current position of drone. This value must be updated each time in your whycon callback
		# [x,y,z]
		self.drone_position = [0.0,0.0,0.0]	

		# [x_setpoint, y_setpoint, z_setpoint]
		self.setpoint = [0,0,23] # whycon marker at the position of the dummy given in the scene. Make the whycon marker associated with position_to_hold dummy renderable and make changes accordingly


		#Declaring a cmd of message type edrone_msgs and initializing values
		self.cmd = edrone_msgs()
		self.cmd.rcRoll = 1500
		self.cmd.rcPitch = 1500
		self.cmd.rcYaw = 1500
		self.cmd.rcThrottle = 1500
		self.cmd.rcAUX1 = 1500
		self.cmd.rcAUX2 = 1500
		self.cmd.rcAUX3 = 1500
		self.cmd.rcAUX4 = 1500


		#initial setting of Kp, Kd and ki for [roll, pitch, throttle]. eg: self.Kp[2] corresponds to Kp value in throttle axis
		#after tuning and computing corresponding PID parameters, change the parameters
		self.Kp = [0,0,0]
		self.Ki = [0,0,0]
		self.Kd = [0,0,0]


		#-----------------------Add other required variables for pid here ----------------------------------------------

		self.throttle_error=0
		self.roll_error=0
		self.pitch_error=0
		self.throttle_prev_error=0
		self.throttle_sum_error=0
		self.roll_prev_error=0
		self.pitch_prev_error=0
		self.roll_sum_error=0
		self.pitch_sum_error=0

		self.min_throttle=1000
		self.max_throttle=2000
		self.min_roll=1000
		self.max_roll=2000
		self.min_pitch=1000
		self.max_pitch=2000
		self.flag=0
		# Hint : Add variables for storing previous errors in each axis, like self.prev_values = [0,0,0] where corresponds to [pitch, roll, throttle]		#		 Add variables for limiting the values like self.max_values = [2000,2000,2000] corresponding to [roll, pitch, throttle]
		#													self.min_values = [1000,1000,1000] corresponding to [pitch, roll, throttle]
		#																	You can change the upper limit and lower limit accordingly. 
		#----------------------------------------------------------------------------------------------------------

		# # This is the sample time in which you need to run pid. Choose any time which you seem fit. Remember the stimulation step time is 50 ms
		self.sample_time = 0.060 # in seconds







		# Publishing /drone_command, /alt_error, /pitch_error, /roll_error
		self.command_pub = rospy.Publisher('/drone_command', edrone_msgs, queue_size=1)
		#------------------------Add other ROS Publishers here-----------------------------------------------------

		self.throttle_error_pub=rospy.Publisher('/alt_error',Float64,queue_size=1)
		self.pitch_error_pub=rospy.Publisher('/pitch_error',Float64,queue_size=1)
		self.roll_error_pub=rospy.Publisher('/roll_error',Float64,queue_size=1)






		#-----------------------------------------------------------------------------------------------------------


		# Subscribing to /whycon/poses, /pid_tuning_altitude, /pid_tuning_pitch, pid_tuning_roll
		rospy.Subscriber('whycon/poses', PoseArray, self.whycon_callback)
		rospy.Subscriber('/pid_tuning_altitude',PidTune,self.altitude_set_pid)
		#-------------------------Add other ROS Subscribers here----------------------------------------------------

		rospy.Subscriber('/pid_tuning_pitch',PidTune,self.pitch_set_pid)
		rospy.Subscriber('/pid_tuning_roll',PidTune,self.roll_set_pid)
		





		#------------------------------------------------------------------------------------------------------------

		self.arm() # ARMING THE DRONE


	# Disarming condition of the drone
	def disarm(self):
		self.cmd.rcAUX4 = 1100
		self.command_pub.publish(self.cmd)
		rospy.sleep(1)


	# Arming condition of the drone : Best practise is to disarm and then arm the drone.
	def arm(self):

		self.disarm()

		self.cmd.rcRoll = 1500
		self.cmd.rcYaw = 1500
		self.cmd.rcPitch = 1500
		self.cmd.rcThrottle = 1000
		self.cmd.rcAUX4 = 1500
		self.command_pub.publish(self.cmd)	# Publishing /drone_command
		rospy.sleep(1)



	# Whycon callback function
	# The function gets executed each time when /whycon node publishes /whycon/poses 
	def whycon_callback(self,msg):
		self.drone_position[0] = msg.poses[0].position.x
		self.drone_position[1] = msg.poses[0].position.y
		self.drone_position[2] = msg.poses[0].position.z



		#--------------------Set the remaining co-ordinates of the drone from msg----------------------------------------------

	



		
		#---------------------------------------------------------------------------------------------------------------



	# Callback function for /pid_tuning_altitude
	# This function gets executed each time when /tune_pid publishes /pid_tuning_altitude
	def altitude_set_pid(self,alt):
		self.Kp[2] = alt.kp * 0.06 # This is just for an example. You can change the ratio/fraction value accordingly
		self.Ki[2] = alt.kp * 0.008
		self.Kd[2] = alt.kp * 0.3

	#----------------------------Define callback function like altitide_set_pid to tune pitch, roll--------------

	def pitch_set_pid(self,pit):
		self.Kp[1]= pit.Kp * 0.06
		self.Ki[1]= pit.Ki * 0.008
		self.Kd[1]= pit.Kd * 0.3


	def roll_set_pid(self,rol):
		self.Kp[0]= rol.Kp * 0.06
		self.Ki[0]= rol.Ki * 0.008
		self.Kd[0]= rol.Kd * 0.3
















	#----------------------------------------------------------------------------------------------------------------------


	def pid(self):
	#-----------------------------Write the PID algorithm here--------------------------------------------------------------

	# Steps:
	# 	1. Compute error in each axis. eg: error[0] = self.drone_position[0] - self.setpoint[0] ,where error[0] corresponds to error in x...
	#	2. Compute the error (for proportional), change in error (for derivative) and sum of errors (for integral) in each axis. Refer "Understanding PID.pdf" to understand PID equation.
	#	3. Calculate the pid output required for each axis. For eg: calcuate self.out_roll, self.out_pitch, etc.
	#	4. Reduce or add this computed output value on the avg value ie 1500. For eg: self.cmd.rcRoll = 1500 + self.out_roll. LOOK OUT FOR SIGN (+ or -). EXPERIMENT AND FIND THE CORRECT SIGN
	#	5. Don't run the pid continously. Run the pid only at the a sample time. self.sampletime defined above is for this purpose. THIS IS VERY IMPORTANT.
	#	6. Limit the output value and the final command value between the maximum(2000) and minimum(1000)range before publishing. For eg : if self.cmd.rcPitch > self.max_values[1]:
	#																														self.cmd.rcPitch = self.max_values[1]
	#	7. Update previous errors.eg: self.prev_error[1] = error[1] where index 1 corresponds to that of pitch (eg)
	#	8. Add error_sum
		if self.drone_position[0]>-0.2 and self.drone_position[0]<0.2 and self.drone_position[1]>-0.2 and self.drone_position[1]<0.2 and self.drone_position[2]>22.8 and self.drone_position[2]<23.2 and self.flag==0 :
			
			self.setpoint[0]=2
			self.setpoint[1]=0
			self.setpoint[2]=23
			print("set point 1 is reached")
			self.flag=1			
		if	self.drone_position[0]>1.8 and self.drone_position[0]<2.2 and self.drone_position[1]>-0.2 and self.drone_position[1]<0.2 and self.drone_position[2]>22.8 and self.drone_position[2]<23.2 and self.flag==1 :
			self.setpoint[0]=2
			self.setpoint[1]=2
			self.setpoint[2]=23
			print("set point 2 is reached")
			self.flag=2					 
		if	self.drone_position[0]>1.8 and self.drone_position[0]<2.2 and self.drone_position[1]>1.8 and self.drone_position[1]<2.2 and self.drone_position[2]>22.8 and self.drone_position[2]<23.2 and self.flag==2 :
			self.setpoint[0]=-2
			self.setpoint[1]=2
			self.setpoint[2]=23
			print("set point 3 is reached")
			self.flag=3	
		if	self.drone_position[0]>-2.2 and self.drone_position[0]<-1.8 and self.drone_position[1]>1.8 and self.drone_position[1]<2.2 and self.drone_position[2]>22.8 and self.drone_position[2]<23.2 and self.flag==3 :
			self.setpoint[0]=-2
			self.setpoint[1]=-2
			self.setpoint[2]=23
			print("set point 4 is reached")
			self.flag=4	
		if	self.drone_position[0]>-2.2 and self.drone_position[0]<-1.8 and self.drone_position[1]>-2.2 and self.drone_position[1]<-1.8 and self.drone_position[2]>22.8 and self.drone_position[2]<23.2 and self.flag==4 :
			self.setpoint[0]=2
			self.setpoint[1]=-2
			self.setpoint[2]=23
			print("set point 5 is reached")
			self.flag=5
		if	self.drone_position[0]>1.8 and self.drone_position[0]<2.2 and self.drone_position[1]>-2.2 and self.drone_position[1]<-1.8 and self.drone_position[2]>22.8 and self.drone_position[2]<23.2 and self.flag==5 :
			self.setpoint[0]=2
			self.setpoint[1]=0
			self.setpoint[2]=23
			print("set point 6 is reached")
			self.flag=6
		if	self.drone_position[0]>1.8 and self.drone_position[0]<2.2 and self.drone_position[1]>-0.2 and self.drone_position[1]<0.2 and self.drone_position[2]>22.8 and self.drone_position[2]<23.2 and self.flag==6 :
			self.setpoint[0]=0
			self.setpoint[1]=0
			self.setpoint[2]=23
			print("set point 7 is reached")
			self.flag=7					
		def throttle_pid():
			self.throttle_error=-(self.setpoint[2]-self.drone_position[2])
			self.cmd.rcThrottle=int(1500+self.throttle_error*1000*0.06+(self.throttle_error-self.throttle_prev_error)*800*0.3+ self.throttle_sum_error*0)
			if self.cmd.rcThrottle>self.max_throttle:
				self.cmd.rcThrottle=self.max_throttle
			if self.cmd.rcThrottle<self.min_throttle:
				self.cmd.rcThrottle=self.min_throttle
			self.throttle_prev_error=self.throttle_error
			self.throttle_sum_error+=self.throttle_error
		def pitch_pid():
			self.pitch_error=(self.setpoint[1]-self.drone_position[1])
			self.cmd.rcPitch=int(1500-(self.pitch_error*40*0.06+(self.pitch_error-self.pitch_prev_error)*60*0.3+ self.pitch_sum_error*0)/self.sample_time)
			if self.cmd.rcPitch>self.max_pitch:
				self.cmd.rcPitch=self.max_pitch
			if self.cmd.rcPitch<self.min_pitch:
				self.cmd.rcPitch=self.min_pitch
			
			self.pitch_prev_error=self.pitch_error
			self.pitch_sum_error+=self.pitch_error
		def roll_pid():
			self.roll_error=(self.setpoint[0]-self.drone_position[0])
			self.cmd.rcRoll=int(1500+(self.roll_error*40*0.06+(self.roll_error-self.roll_prev_error)*60*0.3+ self.roll_sum_error*0)/self.sample_time)
			if self.cmd.rcRoll>self.max_roll:
				self.cmd.rcRoll=self.max_roll
			if self.cmd.rcRoll<self.min_roll:
				self.cmd.rcRoll=self.min_roll

			self.roll_prev_error=self.roll_error
			self.roll_sum_error+=self.roll_error
		throttle_pid()
		roll_pid()
		pitch_pid()
		
		
		
		

		

        






	#------------------------------------------------------------------------------------------------------------------------


		
		self.command_pub.publish(self.cmd)
		self.throttle_error_pub.publish(self.throttle_error)
		self.roll_error_pub.publish(self.roll_error)
		self.pitch_error_pub.publish(self.pitch_error)

if __name__ == '__main__':	
	e_drone = Edrone()
	r = rospy.Rate(1/e_drone.sample_time) #specify rate in Hz based upon your desired PID sampling time, i.e. if desired sample time is 33ms specify rate as 30Hz
	while not rospy.is_shutdown():
		e_drone.pid()
		r.sleep()