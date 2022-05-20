#!/usr/bin/env python
#coding=utf-8
"""
    moveit_ik_demo.py - Version 0.1 2014-01-14
    Use inverse kinemtatics to move the end effector to a specified pose
    
    Created for the Pi Robot Project: http://www.pirobot.org
    Copyleft (c) 2014 Patrick Goebel.  All lefts reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.5
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:
    
    http://www.gnu.org/licenses/gpl.html
"""

#from sqlalchemy import false
import rospy, sys
import moveit_commander
import os
#print('Hello,'+os.environ.get('ROS_MASTER_URI')+'!')
#import moveit_commander
import tf
import argparse
import math
import numpy as np
from math import pi
import time
import copy
from moveit_msgs.msg import RobotTrajectory,DisplayTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from threading import Lock, Event

#from franka_msgs.srv import SetCartesianImpedance, \
#    SetCartesianImpedanceRequest, \
#    SetCartesianImpedanceResponse

#from controller_manager_msgs.srv import SwitchController,SwitchControllerRequest,SwitchControllerResponse

import actionlib
from geometry_msgs.msg import PoseStamped, Pose
from tf.transformations import euler_from_quaternion, quaternion_from_euler,quaternion_multiply,quaternion_from_matrix,quaternion_matrix
#from autolab_core import RigidTransform,transformations
#from pyquaternion import Quaternion
from gpg.msg import GraspConfig,GraspConfigList
#from franka_gripper.msg import GraspAction, GraspGoal
#from franka_gripper.msg import GraspEpsilon
#解析命令行参数
parser = argparse.ArgumentParser(description='Panda go grasp')
parser.add_argument('--test',type=int, default=1)  #设置同时处理几个场景
parameters,unknow =parser.parse_known_args()



class MoveItDemo:
    def __init__(self):
        #初始化moveit的 API接口
        moveit_commander.roscpp_initialize(sys.argv)
        #初始化ros节点 名为ur_grasp
        rospy.init_node('ur_grasp', anonymous=True)
        rospy.set_param("/robot_state", "Initializing")
        rospy.loginfo("Robot  initializing")


        #构建tf发布器
        self.tf_broadcaster=tf.TransformBroadcaster()

        self.grasp_config=GraspConfig()

        #创建多用途的TF监听器
        self.tf_listener = tf.TransformListener()
        #变换关系正确读取的标志位
        get_transform=False
        #等待并获取正确的tf变换关系
        while not get_transform:
            try:
                if parameters.test==1:
                    get_transform = True
                    #print(parameters.test)
                    rospy.loginfo("==================Test mode====================")
                else:
                    self.tf_listener.waitForTransform('/kinect2_link', '/base', rospy.Time(), rospy.Duration(5.0))  
                    #相机坐标系kinect2_link相对于base坐标系位姿
                    self.btc_trans, self.btc_quater = self.tf_listener.lookupTransform('/base', '/kinect2_link', rospy.Time(0))
                    #将trans转换成为ndarry
                    self.btc_trans=np.array(self.btc_trans)
                    self.btc_quater= np.array(self.btc_quater)
                    get_transform = True
                    rospy.loginfo("got transform complete")
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                raise SystemError("got transform failed")



        # 初始化场景对象
        self.scene = moveit_commander.PlanningSceneInterface()
        #为场景添加桌子，防止机械臂碰撞桌面
        self.add_table()
        rospy.sleep(2)
        # 创建机械臂规划组对象
        self.ur_arm = moveit_commander.MoveGroupCommander('manipulator')
        #创建机械手规划对象
        #self.panda_hand=moveit_commander.MoveGroupCommander('hand')
        #
        self.ur_arm.set_max_acceleration_scaling_factor(0.1)
        self.ur_arm.set_max_velocity_scaling_factor(0.2)
        #通过此发布器发布规划的轨迹
        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                               DisplayTrajectory,
                                               queue_size=20)
        # 获取末端执行器名称
        self.end_effector_link = self.ur_arm.get_end_effector_link()
        rospy.loginfo("End effector detected {}".format(self.end_effector_link))         

        # 设置允许机械臂末位姿的错误余量
        self.ur_arm.set_goal_position_tolerance(0.01)#1cm
        self.ur_arm.set_goal_orientation_tolerance(0.05)#

        #不允许规划失败重规划,规划时间只允许5秒钟,否则很浪费时间
        self.ur_arm.allow_replanning(False)
        self.ur_arm.set_planning_time(5)
        
        # 设置ur5的初始姿态，和预备姿态
        self.initial_joints = [0.05453820813327058, -2.4560545497677118, 2.139804820148519, -1.3019487187830345, -1.552458554959968, -0.3681200497067472]
        self.ready_joints = [0.05453820813327058, -2.4560545497677118, 2.139804820148519, -1.3019487187830345, -1.552458554959968, -0.3681200497067472]
        #移动到home
        self.move_to_joints(self.ur_arm,self.ready_joints,tag="reday pose")
        #张开夹爪
        #self.set_gripper(0.078,epsilon=0.01)#张开8cm
        rospy.set_param("/robot_state", "ready")
        rospy.loginfo("Ready to grasp, initial pose")

        ######################开始等待接收夹爪姿态#########################
        rospy.loginfo("Waiting for gripper pose")
        self.callback_done=False

        if parameters.test==1:#测试模式
            self.grasp_test()
        else:
            rospy.Subscriber('best_grasp', GraspConfig, self.Callback,queue_size=1)

        #######################执行抓取####################################
        while not rospy.is_shutdown():
            #等待回调函数处理完
            if self.callback_done:
                self.callback_done=False
                #rospy.set_param("/robot_state", "ready")
                #continue
            else:
                rospy.sleep(0.5)
                continue

            

            #移动至预抓取姿态
            rospy.set_param("/robot_state", "moving")
            rospy.loginfo('Move to pre_grasp pose')
            self.ur_arm.set_start_state_to_current_state()  #以当前姿态作为规划起始点
            success=self.ur_arm.go(self.pre_grasp_pose_wrist3,wait=True)
            self.ur_arm.stop()
            self.ur_arm.clear_pose_targets()
            
            if not success:
                rospy.loginfo('Failed to move to pre_grasp pose!')
                rospy.sleep(1)
                rospy.set_param("/robot_state", "ready")
                continue

            rospy.loginfo('Succeed')

            rospy.sleep(1)#等待机械臂稳定


            #再设置当前姿态为起始姿态
            self.ur_arm.set_start_state_to_current_state()  
            #
            waypoints = []
            wpose=self.ur_arm.get_current_pose().pose
            wpose.position.x=  self.grasp_pose_wrist3.position.x
            wpose.position.y=  self.grasp_pose_wrist3.position.y
            wpose.position.z=  self.grasp_pose_wrist3.position.z
            waypoints.append(copy.deepcopy(wpose))

            #规划从当前位姿，保持姿态，转移到目标夹爪姿态的路径
            (plan, fraction) = self.ur_arm.compute_cartesian_path(
                waypoints,   # waypoints to follow
                0.01,        # eef_step
                0.0)         # jump_threshold
             ##显示轨迹
            display_trajectory = DisplayTrajectory()
            display_trajectory.trajectory_start = self.ur_arm.get_current_state()
            display_trajectory.trajectory.append(plan)
            display_trajectory_publisher.publish(display_trajectory)

            #执行,并等待这个轨迹执行成功
            new_plan=self.scale_trajectory_speed(plan,0.2)
            self.ur_arm.execute(new_plan,wait=True)

            #执行抓取
            rospy.loginfo("Start to grasp")
            #self.set_gripper(0.01,epsilon=0.4)#张开3cm
            rospy.sleep(1)

            ####################抓取完后撤####################
            waypoints = []
            wpose=self.ur_arm.get_current_pose().pose
            wpose.position.x=  self.pre_grasp_pose_wrist3.position.x
            wpose.position.y=  self.pre_grasp_pose_wrist3.position.y
            wpose.position.z=  self.pre_grasp_pose_wrist3.position.z
            waypoints.append(copy.deepcopy(wpose))
            
            #规划从当前位姿，保持姿态，转移到目标夹爪姿态的路径
            (plan, fraction) = self.ur_arm.compute_cartesian_path(
                waypoints,   # waypoints to follow
                0.01,        # eef_step
                0.0)         # jump_threshold
            #显示轨迹
            display_trajectory = DisplayTrajectory()
            display_trajectory.trajectory_start = self.ur_arm.get_current_state()
            display_trajectory.trajectory.append(plan)
            display_trajectory_publisher.publish(display_trajectory)
            #执行,并等待后撤成功
            new_plan=self.scale_trajectory_speed(plan,0.6)
            self.ur_arm.execute(new_plan,wait=True)

            ######################移动到预备姿态############################
            self.move_to_joints(self.ur_arm,self.ready_joints,tag="ready pose")
            #self.set_gripper(0.08,epsilon=0.4)#张开8cm
            rospy.set_param("/robot_state", "ready")
            rospy.loginfo("Ready to grasp, ready pose")
            rospy.sleep(2)
            if parameters.test==1:#测试模式
                self.callback_done=True
                rospy.set_param("/robot_state", "moving")
                self.move_to_joints(self.ur_arm,self.initial_joints)
                rospy.set_param("/robot_state", "ready")


        # Shut down MoveIt cleanly
        moveit_commander.roscpp_shutdown()
        
        # Exit MoveIt
        moveit_commander.os._exit(0)

    def lookupTransform(self,tf_listener, target, source):
        tf_listener.waitForTransform(target, source, rospy.Time(), rospy.Duration(4.0)) #等待时间为4秒

        trans, rot = tf_listener.lookupTransform(target, source, rospy.Time())
        euler = tf.transformations.euler_from_quaternion(rot)

        source_target = tf.transformations.compose_matrix(translate = trans, angles = euler)
        return source_target
    def getTfFromMatrix(self,matrix):
        scale, shear, angles, trans, persp = tf.transformations.decompose_matrix(matrix)
        return trans, tf.transformations.quaternion_from_euler(*angles), angles


    def quater_multi_vec(self,quater,vec):
        quater_=tf.transformations.quaternion_inverse(quater)
        vec_quater=np.c_[vec,[0]]
        temp=quaternion_multiply(quater,vec_quater)
        temp=quaternion_multiply(temp,quater_)
        return temp[:3]

    def move_to_joints(self,group,joints,tag="initial pose"):
        #先从Initial 移动到HOME
        case,plan  = self.planJointGoal(group,joints)#返回真  就是找到轨迹    
        if case==2:
            rospy.loginfo("Move to {}".format(tag))
            group.execute(plan,wait=True)
        elif case==1:
            rospy.loginfo("Already at {}".format(tag))

        else:
            raise SystemError("Home pose  trajectory  not found")

    def planJointGoal(self,movegroup,joint_goal,lable='Next'):
        current_joint = movegroup.get_current_joint_values()
        dis_pose =np.linalg.norm(np.array(joint_goal)-np.array(current_joint))
        #print(current_joint)
        #print(joint_goal)
        if dis_pose<0.008:
            return 1,None #已经到位
        else:
            movegroup.set_joint_value_target(joint_goal)
            plan = movegroup.plan()
            if not plan.joint_trajectory.points:
                return 0,plan
            else:#执行规划
                return 2,plan

    
    def Callback(self,grasp): 
        """根据接收的夹爪抓取姿态，计算预抓取夹爪的位置姿态
        接收的抓取默认以相机坐标系为参考系
        使用的抓取坐标系为典范抓取坐标系(抓取中心点位于两指中心)
        """
        rospy.set_param("/robot_state", "caculating")

        #data是GraspConfigList,data.grasps是GraspConfig[]类型,
        #data.grasps[0]是list中第一个GraspConfig类型的数据，代表的最优的那个抓取配置
        self.grasp_config=grasp
        #最终抓取姿态
        self.grasp_pose_wrist3=Pose()
        #设置后撤距离(10cm)
        dis =0.15

        #LTE      panda_EE(夹爪)相对于link8坐标系的位姿(机械臂固定参数值)
        lte_trans=np.array([0.0000,0.0000,0.1034])
        lte_quater = np.array([0.0000,0.0000,-0.38268,0.92388])
        lte_rot = quaternion_matrix(lte_quater)#[4,4]

        #预抓取姿态
        self.pre_grasp_pose_wrist3=Pose()

        #以下是读取grasp的pose，需要注意的是，此时pose的参考系为相机坐标系
        top_center = np.array([self.grasp_config.position.x,self.grasp_config.position.y,self.grasp_config.position.z])
        approach=np.array([self.grasp_config.approach.x,self.grasp_config.approach.y,self.grasp_config.approach.z])#接近轴
        binormal=np.array([self.grasp_config.binormal.x,self.grasp_config.binormal.y,self.grasp_config.binormal.z])#合并轴
        axis=np.array([self.grasp_config.axis.x,self.grasp_config.axis.y,self.grasp_config.axis.z])#
        #进行方向向量归一化
        approach=approach/np.linalg.norm(approach)
        binormal=binormal/np.linalg.norm(binormal)
        axis=axis/np.linalg.norm(axis)
        #得到典范抓取坐标系在相机坐标系下的位置姿态
        ctg_rot_ = np.concatenate((approach,binormal,axis),axis=0).reshape(3,3).T#3*3
        #panda_EE坐标系和gpg使用的典范抓取坐标系不同，由于获取的是gpg标准的抓取坐标系
        #在这里转换为panda_EE 使用的坐标系形式，实质上就是绕着gpg坐标系y轴旋转90度
        gteg_rot_ = np.array([0,0,1,0,1,0,-1,0,0]).reshape(3,3)
        #得到panda_EE抓取坐标系相对于kinect的姿态
        cteg_rot_ = ctg_rot_.dot(gteg_rot_)
        cteg_rot=np.identity(4)
        cteg_rot[0:3,0:3] = cteg_rot_
        cteg_quater = quaternion_from_matrix(cteg_rot)
        cteg_trans = top_center

        #相机在基座坐标系下的位姿
        btc_rot = quaternion_matrix(self.btc_quater)
        btc_trans = self.btc_trans


        #BTEg    最终抓取状态时，panda_EE相对于基座的位姿
        bteg_trans =btc_trans+btc_rot[:3,:3].dot(cteg_trans)
        bteg_rot = btc_rot.dot(cteg_rot)#4*4
        bteg_quater=quaternion_from_matrix(bteg_rot)

        #BTEp    预抓取状态时，panda_EE相对于基座的位姿
        btep_trans = bteg_trans - bteg_rot[:3,2]*dis #[3,]
        btep_rot = bteg_rot 

        #BTLg    最终抓取状态时，panda_link8相对于基座的位姿
        btlg_rot = bteg_rot.dot(lte_rot.T)#姿态
        btlg_trans = bteg_trans - btlg_rot[:3,:3].dot(lte_trans)#位置
        btlg_quater=quaternion_from_matrix(btlg_rot)

        self.grasp_pose_wrist3.position.x = btlg_trans[0]
        self.grasp_pose_wrist3.position.y = btlg_trans[1]
        self.grasp_pose_wrist3.position.z = btlg_trans[2]
        self.grasp_pose_wrist3.orientation.x = btlg_quater[0]
        self.grasp_pose_wrist3.orientation.y = btlg_quater[1]
        self.grasp_pose_wrist3.orientation.z = btlg_quater[2]
        self.grasp_pose_wrist3.orientation.w = btlg_quater[3]

        #BTLp    预抓取状态时，panda_link8相对于基座的位姿
        self.pre_grasp_pose_wrist3 = copy.deepcopy(self.grasp_pose_wrist3)

        btlp_rot = btlg_rot
        btlp_trans = btep_trans - btlp_rot[:3,:3].dot(lte_trans)
        self.pre_grasp_pose_wrist3.position.x = btlp_trans[0]
        self.pre_grasp_pose_wrist3.position.y = btlp_trans[1]
        self.pre_grasp_pose_wrist3.position.z = btlp_trans[2]

        #发布CTEg    最终抓取状态时，panda_EE相对于基座的位姿
        self.tf_broadcaster.sendTransform(
            self.btc_trans,
            self.btc_quater,
            rospy.Time.now(),
            "kinect2",
            "panda_link0")        

        #发布BTEg    最终抓取状态时，panda_EE相对于基座的位姿
        self.tf_broadcaster.sendTransform(
            bteg_trans,
            bteg_quater,
            rospy.Time.now(),
            "base2grasp",
            "panda_link0")        
        #发布BTEp    预抓取状态时，panda_EE相对于基座的位姿
        self.tf_broadcaster.sendTransform(
            btep_trans,
            bteg_quater,#与抓取姿态相同
            rospy.Time.now(),
            "base2pre",
            "panda_link0")   
        #标志回调函数处理完毕
        self.callback_done=True 

    def grasp_test(self): 
        """
        给定AG95坐标系在base坐标系下的最终抓取姿态  BTEg
        分别计算:
        wrist_3_link坐标系相对于base_link坐标系下的预抓取姿态  BTLp
        wrist_3_link坐标系相对于base_link坐标系下的最终抓取姿态  BTLg
        """
        #设置夹爪后撤距离(10cm)
        dis =0.15

        #BTLg    最终抓取状态时，wrist_3_link相对于base_link的位姿
        self.grasp_pose_wrist3=Pose()
        #BTLp    预抓取状态时，wrist_3_link相对于base_link的位姿
        self.pre_grasp_pose_wrist3=Pose()


        #LTE      AG95(夹爪)相对于wrist_3_link坐标系的位姿(机械臂固定参数值)
        #暂时使用tool0_controller坐标系代替
        lte_trans=np.array([0.0000,0.0000,0.1034]) #偏移
        lte_quater = np.array([0.0000,0.0000,0.0000,1.0000]) #姿态
        lte_rot = quaternion_matrix(lte_quater)#[4,4]
        #lte_rot = lte_rot[:3,:3]
        #BTEg    最终抓取状态时，AG95相对于基座的位姿
        bteg_trans =np.array([0.58166,0.08053,0.11798])
        bteg_quater=np.array([-0.69264,0.72096,-0.0053162,-0.020851])#将姿态转换为四元数形式
        bteg_rot = quaternion_matrix(bteg_quater)
        #bteg_rot = bteg_rot[:3,:3]#截取旋转矩阵[3,3]
        #BTEp    预抓取状态时，AG95相对于基座的位姿
        btep_trans = bteg_trans - bteg_rot[:3,2]*dis #[3,]
        btep_rot = bteg_rot 

        #BTLg    最终抓取状态时，wrist_3_link相对于基座的位姿
        btlg_rot = bteg_rot.dot(lte_rot.T)#姿态
        btlg_trans = bteg_trans - btlg_rot[:3,:3].dot(lte_trans)#位置
        btlg_quater=quaternion_from_matrix(btlg_rot)

        self.grasp_pose_wrist3.position.x = btlg_trans[0]
        self.grasp_pose_wrist3.position.y = btlg_trans[1]
        self.grasp_pose_wrist3.position.z = btlg_trans[2]
        self.grasp_pose_wrist3.orientation.x = btlg_quater[0]
        self.grasp_pose_wrist3.orientation.y = btlg_quater[1]
        self.grasp_pose_wrist3.orientation.z = btlg_quater[2]
        self.grasp_pose_wrist3.orientation.w = btlg_quater[3]

        #BTLp    预抓取状态时，wrist_3_link相对于基座的位姿
        self.pre_grasp_pose_wrist3 = copy.deepcopy(self.grasp_pose_wrist3)

        btlp_rot = btlg_rot
        btlp_trans = btep_trans - btlp_rot[:3,:3].dot(lte_trans)
        self.pre_grasp_pose_wrist3.position.x = btlp_trans[0]
        self.pre_grasp_pose_wrist3.position.y = btlp_trans[1]
        self.pre_grasp_pose_wrist3.position.z = btlp_trans[2]


        #发布BTEg    最终抓取状态时，AG95相对于基座的位姿
        self.tf_broadcaster.sendTransform(
            bteg_trans,
            bteg_quater,
            rospy.Time.now(),
            "base2grasp",
            "base")        
        #发布BTEp    预抓取状态时，panda_EE相对于基座的位姿
        self.tf_broadcaster.sendTransform(
            btep_trans,
            bteg_quater,#与抓取姿态相同
            rospy.Time.now(),
            "base2pre",
            "base")   

        #标志回调函数处理完毕
        self.callback_done=True 


    def scale_trajectory_speed(self,traj,spd=0.1):
        new_traj = RobotTrajectory()
        new_traj = traj

        n_joints = len(traj.joint_trajectory.joint_names)
        n_points = len(traj.joint_trajectory.points)

        #spd = 3.0

        points = list(traj.joint_trajectory.points)

        for i in range(n_points):
            point = JointTrajectoryPoint()
            point.time_from_start = traj.joint_trajectory.points[i].time_from_start / spd
            point.velocities = list(traj.joint_trajectory.points[i].velocities)
            point.accelerations = list(traj.joint_trajectory.points[i].accelerations)
            point.positions = traj.joint_trajectory.points[i].positions

            for j in range(n_joints):
                point.velocities[j] = point.velocities[j] * spd
                point.accelerations[j] = point.accelerations[j] * spd

            points[i] = point

        new_traj.joint_trajectory.points = points     
        return   new_traj

    def add_table(self):
        """为场景中添加抓取桌面，防止机械臂与桌子发生碰撞
        """
        #清除场景可能存在的遗留物体
        self.scene.remove_world_object('table') 
        #设置桌面尺寸      x  y   z
        table_size = [0.6, 1.2, 0.01]
        #设置桌子的位置姿态
        table_pose = PoseStamped()
        table_pose.header.frame_id = 'panda_link0'
        table_pose.pose.position.x = 0.55
        table_pose.pose.position.y = 0.0
        table_pose.pose.position.z = 0.025
        table_pose.pose.orientation.w = 1.0
        # 将table加入场景当中
        self.scene.add_box('table', table_pose, table_size)

    def set_gripper(self,gripper_width,epsilon=0.0):
        """设置panda 夹爪的开合大小
        gripper_width 最大0.08m
        """
        if gripper_width>0.08 or gripper_width<0.0:
             raise Exception
        #帮助维持夹爪力度
        grasp_epsilon = GraspEpsilon(epsilon,epsilon)
        goal = GraspGoal(width = gripper_width, speed = 0.08,epsilon=grasp_epsilon ,force=5.0)
        self.gripper_client.send_goal(goal )
        self.gripper_client.wait_for_result(rospy.Duration.from_sec(5.0))

        rospy.loginfo("Gripper action completed")



if __name__ == "__main__":
    try:
        MoveItDemo()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Arm tracker node terminated.")

    
    