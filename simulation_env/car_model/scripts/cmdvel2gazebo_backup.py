#!/usr/bin/python3

import rospy
from std_msgs.msg import Float64
import math
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Quaternion
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.msg import ModelState
from gazebo_msgs.msg import ContactsState

class CmdVel2Gazebo:

    def __init__(self):
        rospy.init_node('cmdvel2gazebo', anonymous=True)
        
        self.control_sub = rospy.Subscriber('/mobile_base/cmd_vel', Twist, self.callback)

        self.pub_steerL_front = rospy.Publisher('/mobile_base/front_left_steering_position_controller/command', Float64, queue_size=1)
        self.pub_steerR_front = rospy.Publisher('/mobile_base/front_right_steering_position_controller/command', Float64, queue_size=1)

        self.pub_steerL_rear = rospy.Publisher('/mobile_base/rear_left_steering_position_controller/command', Float64, queue_size=1)
        self.pub_steerR_rear = rospy.Publisher('/mobile_base/rear_right_steering_position_controller/command', Float64, queue_size=1)
        self.pub_frontL = rospy.Publisher('/mobile_base/front_left_velocity_controller/command', Float64, queue_size=1)
        self.pub_frontR = rospy.Publisher('/mobile_base/front_right_velocity_controller/command', Float64, queue_size=1)

        self.pub_rearL = rospy.Publisher('/mobile_base/rear_left_velocity_controller/command', Float64, queue_size=1)
        self.pub_rearR = rospy.Publisher('/mobile_base/rear_right_velocity_controller/command', Float64, queue_size=1)

        self.pub_angle2north = rospy.Publisher('/car_angle2north', Float64, queue_size=1)

        

        # initial velocity and tire angle are 0
        self.x = 0
        self.z = 0

        # car Wheelbase (in m)
        self.L = 1.04

        # car Tread
        self.T_front = 0.62  #轮胎间距
        self.T_rear = 0.62 #1.386

        # how many seconds delay for the dead man's switch
        self.timeout=rospy.Duration.from_sec(0.5);
        self.lastMsg=rospy.Time.now()

        # maximum steer angle of the "inside" tire
        self.maxsteerInside=0.75;

        # turning radius for maximum steer angle just with the inside tire
        # tan(maxsteerInside) = wheelbase/radius --> solve for max radius at this angle
        rMin = (self.L/2.0)/math.tan(self.maxsteerInside);

        # radius of inside tire is rMin, so radius of the ideal middle tire (rIdeal) is rMin+treadwidth/2
        rIdeal = rMin+(self.T_front/2.0)

        # maximum steering angle for ideal middle tire
        # tan(angle) = wheelbase/radius
        self.maxsteer=math.atan2(self.L/2.0,rIdeal) #The maximum steering angle mapped to the front (and rear) wheels of a two wheeled vehicle

        # loop
        rate = rospy.Rate(20) # run at 20Hz
        while not rospy.is_shutdown():
            self.publish()
            rate.sleep()

    def callback(self,data):
        # w = v / r
        self.x = data.linear.x /0.16 #The front and rear wheels have the same speed,self.x is the speed of front(or rear) wheel
        # constrain the ideal steering angle such that the ackermann steering is maxed out

        self.z = max(-self.maxsteer,min(self.maxsteer,data.angular.z))#The steering angle mapped to the front (and rear) wheels of a two wheeled vehicle
        self.lastMsg = rospy.Time.now()

    def publish(self):
        # now that these values are published, we
        # reset the velocity, so that if we don't hear new
        # ones for the next timestep that we time out; note
        # that the tire angle will not change
        # NOTE: we only set self.x to be 0 after 200ms of timeout
        delta_last_msg_time = rospy.Time.now() - self.lastMsg
        msgs_too_old = delta_last_msg_time > self.timeout
        if msgs_too_old:
            self.x = 0
            msgRear = Float64()
            msgRear.data = self.x
            self.pub_rearL.publish(msgRear)
            self.pub_rearR.publish(msgRear)
            self.pub_frontL.publish(msgRear)
            self.pub_frontR.publish(msgRear)
            msgSteer = Float64()
            msgSteer.data = 0
            self.pub_steerL_front.publish(msgSteer)
            self.pub_steerR_front.publish(msgSteer)
            self.pub_steerL_rear.publish(msgSteer)
            self.pub_steerR_rear.publish(msgSteer)

            return

        # The self.z is the delta angle in radians of the imaginary front wheel of ackerman model.
        if self.z != 0:
            T_rear = self.T_rear
            T_front = self.T_front
            L=self.L/2.0
            # self.v is the linear *velocity*
            r = L/math.fabs(math.sin(self.z))
            r_to_middle = L/math.fabs(math.tan(self.z))

            # turnning left is possitive
            # the radius to the center of the car
            rL_rear = r_to_middle-(math.copysign(1,self.z)*(T_rear/2.0))
            rR_rear = r_to_middle+(math.copysign(1,self.z)*(T_rear/2.0))
            rL_front = r_to_middle-(math.copysign(1,self.z)*(T_front/2.0))
            rR_front = r_to_middle+(math.copysign(1,self.z)*(T_front/2.0))

            # calculate the angle
            angleL_rear = math.atan2(L,rL_rear)
            angleR_rear = math.atan2(L,rR_rear)
            angleL_front = math.atan2(L,rL_front)
            angleR_front = math.atan2(L,rR_front)

            msgSteerL_rear = Float64()
            msgSteerR_rear = Float64()
            msgSteerL_rear.data = -angleL_rear*math.copysign(1,self.z)
            msgSteerR_rear.data = -angleR_rear*math.copysign(1,self.z)

            msgSteerL_front = Float64()
            msgSteerR_front = Float64()
            msgSteerL_front.data = angleL_front*math.copysign(1,self.z)
            msgSteerR_front.data = angleR_front*math.copysign(1,self.z)

            msgRearL = Float64()
            msgRearR = Float64()
            msgRearL.data = self.x*math.fabs(math.sin(self.z))/math.sin(angleL_rear)
            msgRearR.data = self.x*math.fabs(math.sin(self.z))/math.sin(angleR_rear)
            
            msgFrontL = Float64()
            msgFrontR = Float64()
            msgFrontL.data = self.x*math.fabs(math.sin(self.z))/math.sin(angleL_front)
            msgFrontR.data = self.x*math.fabs(math.sin(self.z))/math.sin(angleR_front)

            self.pub_steerL_rear.publish(msgSteerL_rear)
            self.pub_steerR_rear.publish(msgSteerR_rear)
            self.pub_steerL_front.publish(msgSteerL_front)
            self.pub_steerR_front.publish(msgSteerR_front)
            self.pub_rearL.publish(msgRearL)
            self.pub_rearR.publish(msgRearR)
            self.pub_frontL.publish(msgFrontL)
            self.pub_frontR.publish(msgFrontR)

        else:
            # if we aren't turning
            msgRear = Float64()
            msgRear.data = self.x;

            msgSteer = Float64()
            msgSteer.data = self.z
            self.pub_steerL_front.publish(msgSteer)
            self.pub_steerR_front.publish(msgSteer)
            self.pub_steerL_rear.publish(msgSteer)
            self.pub_steerR_rear.publish(msgSteer)
            self.pub_rearL.publish(msgRear)
            self.pub_rearR.publish(msgRear)
            self.pub_frontL.publish(msgRear)
            self.pub_frontR.publish(msgRear)

if __name__ == '__main__':
    try:
        CmdVel2Gazebo()
    except rospy.ROSInterruptException:
        pass

