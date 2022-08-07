#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist, PoseStamped
from geographic_msgs.msg import GeoPointStamped
from nav_msgs.msg import Odometry
from mavros_msgs.msg import State, PositionTarget
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest, CommandHome

import math
from tf.transformations import euler_from_quaternion

LATITUDE = 13.947494
LONGITUDE = 100.603394

class px4Command:
    def __init__(self):
        self.fly = False
        self.fly_height = 0.75

        self.state = State()
        self.odom_data = Odometry()

        self.sp = PositionTarget()
        # set the flag to use z-position & x,y,yaw-velocity setpoints
        self.sp.type_mask = int('011111100011', 2)
        self.sp.coordinate_frame = 1 #LOCAL_NED
        self.sp.position.z = self.fly_height

        self.hold_x = 0.0
        self.hold_y = 0.0

        rospy.Subscriber("/mavros/state", State, self.stateCb)
        rospy.Subscriber("/cmd_vel", Twist, self.velCb)
        rospy.Subscriber("/mavros/local_position/odom", Odometry, self.odomCb)

        #self.vel_pub = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel_unstamped", Twist, queue_size=1)
        self.gp_origin_pub = rospy.Publisher("/mavros/global_position/set_gp_origin", GeoPointStamped, queue_size=1, latch = True)
        self.local_pose_pub = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=1)
        self.sp_pub = rospy.Publisher("/mavros/setpoint_raw/local", PositionTarget, queue_size=1)

    def stateCb(self, msg):
        self.state = msg

    def odomCb(self, msg):
        self.odom_data = msg

    def velCb(self, msg):
        if self.fly:

            phi = (-1)*euler_from_quaternion([self.odom_data.pose.pose.orientation.x,
                                            self.odom_data.pose.pose.orientation.y,
                                            self.odom_data.pose.pose.orientation.z,
                                            self.odom_data.pose.pose.orientation.w])[2]
            x_dot = msg.linear.x
            y_dot = msg.linear.y
            
            xl_dot = x_dot*math.cos(phi) + y_dot*math.sin(phi)
            yl_dot = y_dot*math.cos(phi) - x_dot*math.sin(phi)

            if((abs(xl_dot) < 0.01) and (abs(yl_dot) < 0.01)):
                self.sp.type_mask = int('011111111000', 2)
                self.sp.position.x = self.hold_x
                self.sp.position.y = self.hold_y
                self.sp.yaw_rate = msg.angular.z
            else:
                self.sp.type_mask = int('011111100011', 2)
                self.sp.velocity.x = xl_dot
                self.sp.velocity.y = yl_dot
                self.sp.yaw_rate = msg.angular.z

                self.hold_x = self.odom_data.pose.pose.position.x
                self.hold_y = self.odom_data.pose.pose.position.y

            self.sp_pub.publish(self.sp)

    def setArm(self, arm):
        rospy.wait_for_service("/mavros/cmd/arming")
        armService = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
        
        arm_cmd = CommandBoolRequest()
        arm_cmd.value = arm

        while(not self.state.armed):
            if(armService.call(arm_cmd).success == True):
                rospy.loginfo("Arming/Disarming")
            rospy.sleep(0.5)

    def setOffboardMode(self):
        rospy.wait_for_service("/mavros/set_mode")
        flightModeService = rospy.ServiceProxy("/mavros/set_mode", SetMode)

        offb_set_mode = SetModeRequest()
        offb_set_mode.custom_mode = 'OFFBOARD'
        
        while(self.state.mode != "OFFBOARD"):
            if(flightModeService.call(offb_set_mode).mode_sent):
                rospy.loginfo("OFFBOARD enabled")
            rospy.sleep(0.5)
        rospy.loginfo("OFFBOARD finished.")

    def setHoldMode(self):
        rospy.wait_for_service("/mavros/set_mode")
        holdService = rospy.ServiceProxy("/mavros/set_mode", SetMode)

        hold_set_mode = SetModeRequest()
        hold_set_mode.custom_mode = 'AUTO.LOITER'

        while(self.state.mode != "AUTO.LOITER"):
            if(holdService.call(hold_set_mode).mode_sent):
                rospy.loginfo("HOLD enabled")
            rospy.sleep(0.5)
        rospy.loginfo("HOLD finished.")
    
    def takeOff(self):
        takeoff_pose = PoseStamped()
        takeoff_pose.pose.position.x = 0.0
        takeoff_pose.pose.position.y = 0.0
        takeoff_pose.pose.position.z = self.fly_height

        rospy.loginfo("Takeoff...")
        while(abs(self.odom_data.pose.pose.position.z-self.fly_height) > 0.05):
            self.local_pose_pub.publish(takeoff_pose)
        rospy.loginfo("Takeoff finished.")
        self.fly = True

    def setLand(self):
        self.fly = False
        rospy.wait_for_service("/mavros/set_mode")
        landService = rospy.ServiceProxy("/mavros/set_mode", SetMode)

        land_set_mode = SetModeRequest()
        land_set_mode.custom_mode = 'AUTO.LAND'

        while(self.state.mode != "AUTO.LAND"):
            if(landService.call(land_set_mode).mode_sent):
                rospy.loginfo("LAND enabled")
            rospy.sleep(0.5)
        rospy.loginfo("LAND finished.")
    
    def setHome(self, lat, long):
        global_origin = GeoPointStamped()
        global_origin.header.stamp = rospy.Time.now()
        global_origin.header.seq = 0
        global_origin.header.frame_id = ""
        global_origin.position.latitude = lat
        global_origin.position.longitude = long
        global_origin.position.altitude = 0

        self.gp_origin_pub.publish(global_origin)

        rospy.wait_for_service("/mavros/cmd/set_home")
        try:
            setHomeService = rospy.ServiceProxy("/mavros/cmd/set_home", CommandHome)
            current_gps = False
            yaw = 0.0
            latitude = lat
            longitude = long
            altitude = 0.0
            setHomeService(current_gps, yaw, latitude, longitude, altitude)
        except rospy.ServiceException as e:
            print("Service set_home call faild: {}. Home could not be set.".format(e))

if __name__ == '__main__':
    rospy.init_node('px4_command')
    print("Initialize px4_command node!")

    vehicle = px4Command()

    rate = rospy.Rate(20)

    while(not rospy.is_shutdown() and not vehicle.state.connected):
        rate.sleep()

    pose = PoseStamped()

    pose.pose.position.x = 0
    pose.pose.position.y = 0
    pose.pose.position.z = 2

    # Send a few setpoints before starting
    for i in range(100):   
        if(rospy.is_shutdown()):
            break

        vehicle.local_pose_pub.publish(pose)
        rate.sleep()

    # Start mission!
    try:
        vehicle.setOffboardMode()
        vehicle.setArm(True)
        vehicle.takeOff()

    except KeyboardInterrupt:
        vehicle.setLand()
        print("Ending the program...")

    rospy.spin()