#!/usr/bin/env python

import rospy
import math
import pymap3d as pm

from tf.transformations import euler_from_quaternion

from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from airsim_ros_pkgs.msg import VelCmd, GPSYaw
from drone_coverage_msgs.srv import SetDroneGoalPose, SetDroneGoalPoseResponse


class Object :
    pass


class PidDroneController :

    def __init__(self) :
        # Initializing this node
        rospy.init_node("pid_drone_controller")
        # Initializing other variables
        self._reached_goal = True
        self._has_odom = False
        self._has_gps_home = False
        self._has_home = False
        self._prev_error = Object()
        self._current_pose = Object()
        self._goal_pose = Object()
        # Loading all the private parameters for this controller
        self._load_pid_params()
        self._load_dynamic_constraints()
        # Initializing the errors
        self._reset_errors()
        # Creating a publisher for the drone velocity
        self._vel_pub = rospy.Publisher(
            "/airsim_node/"+self._drone_name+"/vel_cmd_world_frame", VelCmd, queue_size=1
        )
        # Creating a subscriber for the drone odom
        self._odom_sub = rospy.Subscriber(
            "/airsim_node/"+self._drone_name+"/odom_local_ned", Odometry, queue_size=50, 
            callback=self._on_drone_odometry
        )
        # Creating a subscriber for the gps home
        self._gps_home_sub = rospy.Subscriber(
            "/airsim_node/origin_geo_point", GPSYaw, queue_size=1,
            callback=self._on_gps_home
        )
        # Creating a subscriber for the drone gps
        self._gps_sub = rospy.Subscriber(
            "/airsim_node/"+self._drone_name+"/global_gps", NavSatFix, queue_size=1,
            callback=self._on_drone_gps 
        )
        # Creating a service for the goal position
        self._goal_srv = rospy.Service(
            "/airsim_node/"+self._drone_name+"/local_goal", SetDroneGoalPose, 
            handler=self._on_local_goal_service
        )
        # Creating a timer for handling the update of the velocity
        rospy.Timer(rospy.Duration(self._update_period_sec), self._update_drone_velocity)


    def _load_pid_params(self):
        # Loading the vehicle name this controller is referred to
        self._drone_name = rospy.get_param("~drone_name")
        # Loading all the parameters for the pid controller
        self._kp_x = rospy.get_param("~kp_x")
        self._kp_y = rospy.get_param("~kp_y")
        self._kp_z = rospy.get_param("~kp_z")
        self._kp_yaw = rospy.get_param("~kp_yaw")
        self._kd_x = rospy.get_param("~kd_x")
        self._kd_y = rospy.get_param("~kd_y")
        self._kd_z = rospy.get_param("~kd_z")
        self._kd_yaw = rospy.get_param("~kd_yaw")
        self._goal_xyx_threshold = rospy.get_param("~goal_xyz_threshold")
        self._goal_yaw_threshold = rospy.get_param("~goal_yaw_threshold")
        self._update_period_sec = rospy.get_param("~update_perdiod_sec")
    

    def _load_dynamic_constraints(self):
        # Loading all the parameters for the dynamic constraints
        self._max_vel_horz = rospy.get_param("~max_vel_horz")
        self._max_vel_vert = rospy.get_param("~max_vel_vert")
        self._max_vel_rot = rospy.get_param("~max_vel_rot")


    def _reset_errors(self) :
        # Resetting all the errors related to the goal
        self._prev_error.x = 0.0
        self._prev_error.y = 0.0
        self._prev_error.z = 0.0
        self._prev_error.yaw = 0.0


    def _on_gps_home(self, msg):
        self._has_gps_home = True
        # Storing the current gps home position
        self._gps_home_pose = Object()
        self._gps_home_pose.lat = msg.latitude
        self._gps_home_pose.lon = msg.longitude
        self._gps_home_pose.alt = msg.altitude
        self._gps_home_pose.yaw = msg.yaw
        # The GPS home is always the same, unregistering
        self._gps_home_sub.unregister()
    

    def _on_drone_gps(self, msg):
        # Check if the gps home has been already received
        if not self._has_gps_home :
            return
        # Computing the current position in ned coordinates
        hlat = self._gps_home_pose.lat
        hlon = self._gps_home_pose.lon
        halt = self._gps_home_pose.alt
        home = pm.geodetic2ned(msg.latitude, msg.longitude, msg.altitude, hlat, hlon, halt)
        # Storing the home position and unregistering
        self._has_home = True
        self._home_pose = Object()
        self._home_pose.x = home[0]
        self._home_pose.y = home[1]
        self._home_pose.z = home[2]
        self._home_pose.yaw = self._gps_home_pose.yaw
        self._gps_sub.unregister()
        # Debugging
        ix = "{:.2f}".format(home[0])
        iy = "{:.2f}".format(home[1])
        iz = "{:.2f}".format(home[2])
        rospy.loginfo("[PID "+self._drone_name+"] Obtained initial position at [x:"+ix+" y:"+iy+ " z:"+iz+"]!")
    

    def _on_drone_odometry(self, msg):
        # Waiting for the home position before computing current position
        if not self._has_home :
            return
        # Storing the current position and orientation of the drone
        self._current_pose.x = msg.pose.pose.position.x + self._home_pose.x
        self._current_pose.y = msg.pose.pose.position.y + self._home_pose.y
        self._current_pose.z = msg.pose.pose.position.z + self._home_pose.z
        # Converting the quaternion to yaw angles
        quat = msg.pose.pose.orientation
        quat_expl = [quat.x, quat.y, quat.z, quat.w]
        self._current_pose.yaw = euler_from_quaternion(quat_expl)[2]
        # Debugging
        if not self._has_odom :
            rospy.loginfo("[PID "+self._drone_name+"] First odometry received!")
            self._has_odom = True

    
    def _on_local_goal_service(self, req):
        if not self._has_odom :
            return SetDroneGoalPoseResponse(False)
        # Storing the requested goal position
        self._goal_pose.x = req.x
        self._goal_pose.y = req.y
        self._goal_pose.z = -req.z
        self._goal_pose.yaw = req.yaw
        # Debugging
        rospy.loginfo("[PID "+self._drone_name+"] Requested goal x:"+str(req.x)+" y:"+str(req.y)+ " z:"+str(req.z))
        # There is a new goal to reach
        self._reached_goal = False
        self._reset_errors()
        # The new goal has been successfully set
        return SetDroneGoalPoseResponse(True)
    

    def _check_goal_reached(self):
        # If the goal has already been reached, do nothing
        if self._reached_goal :
            return
        # Computing the error in the position
        error_x = self._goal_pose.x - self._current_pose.x
        error_y = self._goal_pose.y - self._current_pose.y
        error_z = self._goal_pose.z - self._current_pose.z
        diff_xyz = math.sqrt(error_x*error_x + error_y*error_y + error_z*error_z)
        # Computing the error in the yaw
        diff_yaw = angular_dist(self._current_pose.yaw, self._goal_pose.yaw)
        # Checking the thresholds
        if diff_xyz < self._goal_xyx_threshold and diff_yaw < self._goal_xyx_threshold :
            self._reached_goal = True
            rospy.loginfo("[PID "+self._drone_name+"] Goal pose has been reached!")

    
    def _update_drone_velocity(self, event):
        # Check if the current position for the drone is available
        if not self._has_home :
            rospy.logwarn_once("[PID "+self._drone_name+"] Waiting for initial position!")
            return
        if not self._has_odom :
            rospy.logwarn_once("[PID "+self._drone_name+"] Waiting for first odometry!")
            return
        # Check if the current goal has already been reached
        self._check_goal_reached()
        if self._reached_goal :
            return
        # Creating the new velocity for the drone
        vel = self._compute_new_velocity()
        self._enforce_dynamic_constraints(vel)
        # Publishing the new velocity
        self._vel_pub.publish(vel)

    
    def _compute_new_velocity(self):
        # Computing the error between the current pose and target pose
        curr_error = Object()
        curr_error.x = self._goal_pose.x - self._current_pose.x
        curr_error.y = self._goal_pose.y - self._current_pose.y
        curr_error.z = self._goal_pose.z - self._current_pose.z
        curr_error.yaw = angular_dist(self._current_pose.yaw, self._goal_pose.yaw)
        # Computing pid p variables
        p_term_x = self._kp_x * curr_error.x
        p_term_y = self._kp_y * curr_error.y
        p_term_z = self._kp_z * curr_error.z
        p_term_yaw = self._kp_yaw * curr_error.yaw
        # Computing pid d variables
        d_term_x = self._kd_x * self._prev_error.x
        d_term_y = self._kd_y * self._prev_error.y
        d_term_z = self._kd_z * self._prev_error.z
        d_term_yaw = self._kd_yaw * self._prev_error.yaw
        # Storing the current error for the next iteration
        self._prev_error = curr_error
        # Computing the new velocity
        vel = VelCmd()
        vel.twist.linear.x = p_term_x + d_term_x
        vel.twist.linear.y = p_term_y + d_term_y
        vel.twist.linear.z = p_term_z + d_term_z
        vel.twist.angular.z = p_term_yaw + d_term_yaw
        return vel


    def _enforce_dynamic_constraints(self, vel):
        # Computing the magnitude of the horizontal direction of the velocity
        norm_horz = math.sqrt((vel.twist.linear.x * vel.twist.linear.x) + (vel.twist.linear.y * vel.twist.linear.y))
        # Clamping the horizontal velocity
        if norm_horz > self._max_vel_horz :
            vel.twist.linear.x = (vel.twist.linear.x / norm_horz) * self._max_vel_horz 
            vel.twist.linear.y = (vel.twist.linear.y / norm_horz) * self._max_vel_horz
        # Clamping the vertical velocity
        if abs(vel.twist.linear.z) > self._max_vel_vert :
            vel.twist.linear.z = math.copysign(self._max_vel_vert, vel.twist.linear.z)
        # Clamping the rotation velocity
        if abs(vel.twist.angular.z) > self._max_vel_rot :
            vel.twist.angular.z = math.copysign(self._max_vel_rot, vel.twist.angular.z)



def wrap_to_pi(radians) :
    m = int(radians / (2 * math.pi))
    radians = radians - m * 2 * math.pi
    if radians > math.pi :
        radians -= 2.0 * math.pi
    elif radians < -math.pi :
        radians += 2.0 * math.pi
    return radians


def angular_dist(from_rads, to_rads):
    from_rads = wrap_to_pi(from_rads)
    to_rads = wrap_to_pi(to_rads)
    return wrap_to_pi(to_rads - from_rads)


def main():
    PidDroneController()
    rospy.spin()


if __name__ == "__main__":
    main()