#!/bin/python3
# Software License Agreement (BSD License)

import re
import rospy
import numpy as np

import math
import os
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from robo.srv import action
import sys
# from scipy.spatial.transform import Rotation as R
import time

def quaternion_to_rotation_matrix(quat):
    q = quat.copy()
    n = np.dot(q, q)
    if n < np.finfo(q.dtype).eps:
        return np.identity(4)
    q = q * np.sqrt(2.0 / n)
    q = np.outer(q, q)
    rot_matrix = np.array(
    [[1.0 - q[2, 2] - q[3, 3], q[1, 2] + q[3, 0], q[1, 3] - q[2, 0], 0.0],
    [q[1, 2] - q[3, 0], 1.0 - q[1, 1] - q[3, 3], q[2, 3] + q[1, 0], 0.0],
    [q[1, 3] + q[2, 0], q[2, 3] - q[1, 0], 1.0 - q[1, 1] - q[2, 2], 0.0],
    [0.0, 0.0, 0.0, 1.0]],
    dtype=q.dtype)
    return rot_matrix

def get_wp_from_pose(aruco_pose_msg):
    rotation_quaternion = np.asarray([aruco_pose_msg.orientation.w,
    aruco_pose_msg.orientation.x,
    aruco_pose_msg.orientation.y,
    aruco_pose_msg.orientation.z])
    rot_ = quaternion_to_rotation_matrix(rotation_quaternion)
    pos_ = np.asarray([0,0,0.1,0])
    pos_marker = np.asarray([aruco_pose_msg.position.x,
    aruco_pose_msg.position.y,
    aruco_pose_msg.position.z,
    0])
    pos_wp = np.matmul(pos_,rot_)
    pos_wp += pos_marker
    wp_ = pos_wp[:3]
    print("marker : ",pos_marker)
    print("waypoint : ",wp_)
    return wp_

class graspAruco:
    def __init__(self):
        #publish control variables
        self.base_move_position_pub = rospy.Publisher("cmd_position", Twist, queue_size=1)
        self.base_move_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        self.arm_gripper_pub = rospy.Publisher("arm_gripper", Point, queue_size=1)
        self.arm_position_pub = rospy.Publisher("arm_position", Pose, queue_size=1)
        #get the position of sink
        self.image_sub = rospy.Subscriber("/aruco_sink1", Pose, self.sinkCallback1, queue_size = 1)
        self.image_sub = rospy.Subscriber("/aruco_sink2", Pose, self.sinkCallback2, queue_size = 1)
        self.image_sub = rospy.Subscriber("/aruco_sink3", Pose, self.sinkCallback3, queue_size = 1)

        self.server = rospy.Service('place', action, self.handle_place)

        print("ready to accept grasp request")

        #switcher
        self.place_success = False
        self.need_place = False
        self.sink = 0


    def open_gripper(self):
        open_gripper_msg = Point()
        open_gripper_msg.x = 0.0
        open_gripper_msg.y = 0.0
        open_gripper_msg.z = 0.0
        print("open the gripper!")
        self.arm_gripper_pub.publish(open_gripper_msg)

    def close_gripper(self):
        close_gripper_msg = Point()
        close_gripper_msg.x = 1.0
        close_gripper_msg.y = 0.0
        close_gripper_msg.z = 0.0
        print("close the gripper")
        self.arm_gripper_pub.publish(close_gripper_msg)

    def move_arm(self,t_vector):
        print("move_arm", t_vector)
        move_arm_msg = Pose()
        # unit in [cm]
        # in the gripper base frame
        move_arm_msg.position.x = 0.2      # TODO
        move_arm_msg.position.y = 0.12
        move_arm_msg.position.z = 0
        move_arm_msg.orientation.x = 0.0
        move_arm_msg.orientation.y = 0.0
        move_arm_msg.orientation.z = 0.0
        move_arm_msg.orientation.w = 0.0
        print("move the arm to the grasping pose")
        self.arm_position_pub.publish(move_arm_msg)

    def move_arm0(self,t_vector):
        print("move_arm", t_vector)
        move_arm_msg = Pose()
        # unit in [cm]
        # in the gripper base frame
        move_arm_msg.position.x = 0.90      # TODO
        move_arm_msg.position.y = 0.12
        move_arm_msg.position.z = 0
        move_arm_msg.orientation.x = 0.0
        move_arm_msg.orientation.y = 0.0
        move_arm_msg.orientation.z = 0.0
        move_arm_msg.orientation.w = 0.0
        print("move the arm to the grasping pose")
        self.arm_position_pub.publish(move_arm_msg)
 
    def reset_arm(self):            
        reset_arm_msg = Pose()
        reset_arm_msg.position.x = 0.1
        reset_arm_msg.position.y = 0.09
        reset_arm_msg.position.z = 0.0
        reset_arm_msg.orientation.x = 0.0
        reset_arm_msg.orientation.y = 0.0
        reset_arm_msg.orientation.z = 0.0
        reset_arm_msg.orientation.w = 0.0
        print("reset the arm")
        self.arm_position_pub.publish(reset_arm_msg)

    def move_base_x(self, t_vector):
        move_base_msg_x = Twist()
        # unit in [m]
        # in the car frame
        # t_vector is the position of the marker in the camera frame
        #goal = [0.0175, 0.0, 0.2]
        # goal = [0.0175, 0.0, 0.2]
        #distance_x = t_vector[2]-goal[2]
        # # x_move = 0.07
        #tolerance o.05
        #if distance_x <= 0.05:
        #   x_move = 0
        #else:
        x_move = 0.05
        # x_move = 0.5 * distance_x
        
        # print("x_movement", x_move)
        move_base_msg_x.linear.x = x_move
        move_base_msg_x.linear.y = 0.0
        move_base_msg_x.linear.z = 0.0
        move_base_msg_x.angular.x = 0.0
        move_base_msg_x.angular.y = 0.0
        move_base_msg_x.angular.z = 0.0
        print("move the base in x direction with 0.8*dist")
        self.base_move_position_pub.publish(move_base_msg_x)

    #stop the car
    def forward_zero(self):
        vel_cmd = Twist()
        vel_cmd.linear.x = 0.0
        vel_cmd.linear.y = 0.0
        vel_cmd.linear.z = 0.0
        vel_cmd.angular.x = 0.0
        vel_cmd.angular.y = 0.0
        vel_cmd.angular.z = 0.0
        self.base_move_vel_pub.publish(vel_cmd)
        rospy.sleep(0.05)
        
    def forward_minimum_x(self):
        vel_cmd = Twist()
        vel_cmd.linear.x = 0.11
        vel_cmd.linear.y = 0.0
        vel_cmd.linear.z = 0.0
        vel_cmd.angular.x = 0.0
        vel_cmd.angular.y = 0.0
        vel_cmd.angular.z = 0.0
        self.base_move_vel_pub.publish(vel_cmd)
        rospy.sleep(0.05)
        # motor STOP
        vel_cmd.linear.x = 0.0
        vel_cmd.linear.y = 0.0
        vel_cmd.linear.z = 0.0
        vel_cmd.angular.x = 0.0
        vel_cmd.angular.y = 0.0
        vel_cmd.angular.z = 0.0
        self.base_move_vel_pub.publish(vel_cmd)

    def backward_minimum_x(self):
        vel_cmd = Twist()
        vel_cmd.linear.x = -0.11
        vel_cmd.linear.y = 0.0
        vel_cmd.linear.z = 0.0
        vel_cmd.angular.x = 0.0
        vel_cmd.angular.y = 0.0
        vel_cmd.angular.z = 0.0
        self.base_move_vel_pub.publish(vel_cmd)
        rospy.sleep(0.05)
        # motor STOP
        vel_cmd.linear.x = 0.0
        vel_cmd.linear.y = 0.0
        vel_cmd.linear.z = 0.0
        vel_cmd.angular.x = 0.0
        vel_cmd.angular.y = 0.0
        vel_cmd.angular.z = 0.0
        self.base_move_vel_pub.publish(vel_cmd)

    def forward_minimum_y_right(self):
        vel_cmd = Twist()
        vel_cmd.linear.x = 0.0
        vel_cmd.linear.y = -0.11
        vel_cmd.linear.z = 0.0
        vel_cmd.angular.x = 0.0
        vel_cmd.angular.y = 0.0
        vel_cmd.angular.z = 0.0
        self.base_move_vel_pub.publish(vel_cmd)
        rospy.sleep(0.05)
        # motor STOP
        vel_cmd.linear.x = 0.0
        vel_cmd.linear.y = 0.0
        vel_cmd.linear.z = 0.0
        vel_cmd.angular.x = 0.0
        vel_cmd.angular.y = 0.0
        vel_cmd.angular.z = 0.0
        self.base_move_vel_pub.publish(vel_cmd)

    def forward_minimum_y_left(self):
        vel_cmd = Twist()
        vel_cmd.linear.x = 0.0
        vel_cmd.linear.y = 0.11
        vel_cmd.linear.z = 0.0
        vel_cmd.angular.x = 0.0
        vel_cmd.angular.y = 0.0
        vel_cmd.angular.z = 0.0
        self.base_move_vel_pub.publish(vel_cmd)
        rospy.sleep(0.05)
        # motor STOP
        vel_cmd.linear.x = 0.0
        vel_cmd.linear.y = 0.0
        vel_cmd.linear.z = 0.0
        vel_cmd.angular.x = 0.0
        vel_cmd.angular.y = 0.0
        vel_cmd.angular.z = 0.0
        self.base_move_vel_pub.publish(vel_cmd)

    def move_base_velocity_x(self,b_vector = 0.5, duration = 10):
        # b_vector : x_bandwidth
        # duration : multiple of execution_cycle
        execution_cycle = 10.0
        for t in range(int(duration)):
            n_forward_minimun = b_vector * execution_cycle
            n_forward_zero = (1-b_vector) * execution_cycle
            for tt in range(int(n_forward_minimun)):
                self.forward_minimum_x()
            for tt in range(int(n_forward_zero)):
                self.forward_zero()

    def move_base_velocity_back(self,b_vector = 0.5, duration = 10):
        # b_vector : x_bandwidth
        # duration : multiple of execution_cycle
        execution_cycle = 10.0
        for t in range(int(duration)):
            n_forward_minimun = b_vector * execution_cycle
            n_forward_zero = (1-b_vector) * execution_cycle
            for tt in range(int(n_forward_minimun)):
                self.backward_minimum_x()
            for tt in range(int(n_forward_zero)):
                self.forward_zero()

    def move_base_velocity_y_right(self,b_vector = 0.5, duration = 10):
        # b_vector : x_bandwidth
        # duration : multiple of execution_cycle
        execution_cycle = 10.0
        for t in range(int(duration)):
            n_forward_minimun = b_vector * execution_cycle
            n_forward_zero = (1-b_vector) * execution_cycle
            for tt in range(int(n_forward_minimun)):
                self.forward_minimum_y_right()
            for tt in range(int(n_forward_zero)):
                self.forward_zero()

    def move_base_velocity_y_left(self,b_vector = 0.5, duration = 10):
        # b_vector : x_bandwidth
        # duration : multiple of execution_cycle
        execution_cycle = 10.0
        for t in range(int(duration)):
            n_forward_minimun = b_vector * execution_cycle
            n_forward_zero = (1-b_vector) * execution_cycle
            for tt in range(int(n_forward_minimun)):
                self.forward_minimum_y_left()
            for tt in range(int(n_forward_zero)):
                self.forward_zero()

    
    def move_base_y(self,t_vector):
        move_base_msg_y = Twist()
        # unit in [m]
        # in the car frame
        # t_vector is the position of the marker in the camera frame
        #goal = [0.03., 0.0, 0.115]
        # goal = [0.0175, 0.0, 0.2]
        #goal = [0.03, 0.0, 0.2]
        y_move = 0.05
        move_base_msg_y.linear.x = 0.0
        # move_base_msg_y.linear.y = goal[0] - t_vector[0]
        move_base_msg_y.linear.y = y_move
        move_base_msg_y.linear.z = 0.0
        move_base_msg_y.angular.x = 0.0
        move_base_msg_y.angular.y = 0.0
        move_base_msg_y.angular.z = 0.0
        # print("move_base_y {} cm" .format((goal[0] - t_vector[0])*100))
        self.base_move_position_pub.publish(move_base_msg_y)

    def handle_place(self,req):
        if req.place == "1":
            self.need_place = True
            self.sink = 1
            self.place_success = False
            # done = Int16()
            # done.data = 0
            # self.grasp_done_pub.publish(done)
            return True
        elif req.place == "2":
            self.need_place = True
            self.sink = 2
            self.place_success = False
            # done = Int16()
            # done.data = 0
            # self.grasp_done_pub.publish(done)
            return True
        elif req.place == "3":
            self.need_place = True
            self.sink = 3
            self.place_success = False
            # done = Int16()
            # done.data = 0
            # self.grasp_done_pub.publish(done)
            return True
        else:#if req.place == "0":
            self.need_place = False
            self.sink = 0
            return True

    def sinkCallback1(self,data):

        #if already placed 
        if self.sink != 1:
            return
        elif self.place_success == True:
            return
        elif self.need_place == False:
            return

        #gama_x = 0.01
        #gama_y = 0.01
        gama_y = 0.015
        gama_x = 0.015
        
        tvec = [0,0,0]
        tvec[0] = data.position.x
        tvec[1] = data.position.y
        tvec[2] = data.position.z

        # quat = [0,0,0,0]
        # quat[0] = data.orientation.x
        # quat[1] = data.orientation.y
        # quat[2] = data.orientation.z
        # quat[3] = data.orientation.w

        goal = [0.03, 0.0, 0.25]
        distance_in_x = tvec[2] - goal[2]
        distance_in_y = abs(tvec[0] - goal[0])
        if (tvec[0]-goal[0]) > 0:
            move_y_right = True
        else:
            move_y_right = False # move to left
        print("distance in x", distance_in_x)
        print("distance in y", distance_in_y)

        #in ready to place pose range
        if (distance_in_x <= gama_x) and (distance_in_y <= gama_y):
            # step forward
            self.move_base_velocity_x(b_vector = 0.2, duration = 18)
            #self.move_base_velocity_x(b_vector = 0.15, duration = 1)

            print("===== start placing ====")
            self.reset_arm()
            rospy.sleep(0.5)
            self.move_arm0(tvec)
            rospy.sleep(0.5)
            self.move_arm(tvec)
            rospy.sleep(0.5)
            self.open_gripper()
            rospy.sleep(0.5)
            self.reset_arm()
            rospy.sleep(0.5)
            self.close_gripper()
            # self.forward_zero()
            rospy.sleep(0.1)
            print("=====get back ====")
            self.move_base_velocity_back(b_vector = 0.5, duration = 4)
            print("=====place finish ====")

            #switcher
            self.place_success = True
            #self.need_place = False
            #print(need_place)
            self.sink = 0
        
        #adjust position
        else:
            self.move_base_x(tvec)
            self.move_base_y(tvec)
            int(distance_in_x / 0.5)
            if distance_in_x > gama_x:
                #move forward 0.5 
                self.move_base_velocity_x(b_vector = 0.5, duration = 1)
                print("move in x")
                # time.sleep(1)

            if distance_in_y > gama_y:
                #self.move_base_y()
                if move_y_right:
                    
                    #move right for 0.5
                    self.move_base_velocity_y_right(b_vector = 0.5, duration = 1)
                    print("move y to right")
                else:
                    
                    #move left for 0.5
                    self.move_base_velocity_y_left(b_vector = 0.5, duration = 1)
                    print("move y to left")

    def sinkCallback2(self,data):

        #if already placed 
        if self.sink != 2:
            return
        elif self.place_success == True:
            return
        elif self.need_place == False:
            return

        #gama_x = 0.01
        #gama_y = 0.01
        gama_y = 0.015
        gama_x = 0.015
        
        tvec = [0,0,0]
        tvec[0] = data.position.x
        tvec[1] = data.position.y
        tvec[2] = data.position.z

        # quat = [0,0,0,0]
        # quat[0] = data.orientation.x
        # quat[1] = data.orientation.y
        # quat[2] = data.orientation.z
        # quat[3] = data.orientation.w

        goal = [0.03, 0.0, 0.25]
        distance_in_x = tvec[2] - goal[2]
        distance_in_y = abs(tvec[0] - goal[0])
        if (tvec[0]-goal[0]) > 0:
            move_y_right = True
        else:
            move_y_right = False # move to left
        print("distance in x", distance_in_x)
        print("distance in y", distance_in_y)

        #in ready to place pose range
        if (distance_in_x <= gama_x) and (distance_in_y <= gama_y):
            # step forward
            self.move_base_velocity_x(b_vector = 0.2, duration = 18)
            #self.move_base_velocity_x(b_vector = 0.12, duration = 1)

            print("===== start placing ====")
            self.reset_arm()
            rospy.sleep(0.5)
            self.move_arm0(tvec)
            rospy.sleep(0.5)
            self.move_arm(tvec)
            rospy.sleep(0.5)
            self.open_gripper()
            rospy.sleep(0.5)
            self.reset_arm()
            rospy.sleep(0.5)
            self.close_gripper()
            # self.forward_zero()
            rospy.sleep(0.1)
            print("=====get back ====")
            self.move_base_velocity_back(b_vector = 0.5, duration = 4)
            print("=====place finish ====")

            #switcher
            self.place_success = True
            #self.need_place = False
            #print(need_place)
            self.sink = 0
        
        #adjust position
        else:
            self.move_base_x(tvec)
            self.move_base_y(tvec)
            int(distance_in_x / 0.5)
            if distance_in_x > gama_x:
                #move forward 0.5 
                self.move_base_velocity_x(b_vector = 0.5, duration = 1)
                print("move in x")
                # time.sleep(1)

            if distance_in_y > gama_y:
                #self.move_base_y()
                if move_y_right:
                    
                    #move right for 0.5
                    self.move_base_velocity_y_right(b_vector = 0.5, duration = 1)
                    print("move y to right")
                else:
                    
                    #move left for 0.5
                    self.move_base_velocity_y_left(b_vector = 0.5, duration = 1)
                    print("move y to left")

    def sinkCallback3(self,data):
        #if already placed 
        if self.sink != 3:
            return
        elif self.place_success == True:
            return
        elif self.need_place == False:
            return

        #gama_x = 0.01
        #gama_y = 0.01
        gama_y = 0.015
        gama_x = 0.015
        
        tvec = [0,0,0]
        tvec[0] = data.position.x
        tvec[1] = data.position.y
        tvec[2] = data.position.z

        # quat = [0,0,0,0]
        # quat[0] = data.orientation.x
        # quat[1] = data.orientation.y
        # quat[2] = data.orientation.z
        # quat[3] = data.orientation.w

        goal = [0.03, 0.0, 0.25]
        distance_in_x = tvec[2] - goal[2]
        distance_in_y = abs(tvec[0] - goal[0])
        if (tvec[0]-goal[0]) > 0:
            move_y_right = True
        else:
            move_y_right = False # move to left
        print("distance in x", distance_in_x)
        print("distance in y", distance_in_y)

        #in ready to place pose range
        if (distance_in_x <= gama_x) and (distance_in_y <= gama_y):
            # step forward
            self.move_base_velocity_x(b_vector = 0.2, duration = 18)

            print("===== start placing ====")
            self.reset_arm()
            rospy.sleep(0.5)
            self.move_arm0(tvec)
            rospy.sleep(0.5)
            self.move_arm(tvec)
            rospy.sleep(0.5)
            self.open_gripper()
            rospy.sleep(0.5)
            self.reset_arm()
            rospy.sleep(0.5)
            self.close_gripper()
            # self.forward_zero()
            rospy.sleep(0.1)
            print("=====get back ====")
            self.move_base_velocity_back(b_vector = 0.5, duration = 4)
            print("=====place finish ====")

            #switcher
            self.place_success = True
            #self.need_place = False
            #print(need_place)
            self.sink = 0
        #adjust position
        else:
            self.move_base_x(tvec)
            self.move_base_y(tvec)
            int(distance_in_x / 0.5)
            if distance_in_x > gama_x:
                #move forward 0.5 
                self.move_base_velocity_x(b_vector = 0.5, duration = 1)
                print("move in x")
                # time.sleep(1)

            if distance_in_y > gama_y:
                #self.move_base_y()
                if move_y_right:
                    
                    #move right for 0.5
                    self.move_base_velocity_y_right(b_vector = 0.5, duration = 1)
                    print("move y to right")
                else:
                    
                    #move left for 0.5
                    self.move_base_velocity_y_left(b_vector = 0.5, duration = 1)
                    print("move y to left")


       
def main():
    rospy.init_node('grasp_aruco_node', anonymous=True)
    ap = graspAruco()
    print("=====init=====")
    ap.reset_arm()
    rospy.sleep(1)
    print("=====reset arm at beginning=====")
    ap.open_gripper()
    rospy.sleep(1)
    print("=====open gripper at beginning=====")
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
        ap.forward_zero()
        # ap.reset_arm()


if __name__ == '__main__':
    main()
