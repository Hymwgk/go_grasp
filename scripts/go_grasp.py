#!/usr/bin/env python
#coding=utf-8
"""
已测试python环境：2.7.16
author: wgk_ZZU
date: 2022/06/18
abcwgk@gmail.com
"""

#from sqlalchemy import false
import rospy, sys
import rospkg
import rosparam
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
from autolab_core import RigidTransform
#from franka_gripper.msg import GraspAction, GraspGoal
#from franka_gripper.msg import GraspEpsilon
#解析命令行参数
#parser = argparse.ArgumentParser(description='Go grasp')
#parser.add_argument('--test',type=int, default=1)  #设置同时处理几个场景
#parameters,unknow =parser.parse_known_args()



class GoGrasp:
    def __init__(self):
        #初始化moveit的 API接口
        moveit_commander.roscpp_initialize(sys.argv)
        #初始化ros节点 名为ur_grasp
        rospy.init_node('go_grasp', anonymous=True)
        rospy.set_param("/robot_state", "Initializing")
        rospy.loginfo("Robot  initializing")

        #获取当前程序包地址
        rospack = rospkg.RosPack()
        current_path=rospack.get_path('go_grasp')
        
        robot_name = rospy.get_param('~robot_name', 'ur5')        
        #机械臂Moveit规划组名称
        move_group_name = rospy.get_param('~move_group_name', 'manipulator')        
        #机械臂Moveit规划组的基座link坐标系
        base_frame = rospy.get_param('~base_frame', 'base_link')
        #订阅发布GraspConfigList的话题名称
        grasp_list_topic = rospy.get_param('~grasp_list_topic', '/yolov5/detection_grasps')
        #需要说明接收的抓取位姿，是以谁为父坐标系的
        grasp_father_frame = rospy.get_param('~grasp_father_frame', 'camera_color_optical_frame')
        #指定夹爪指尖固连坐标系E与点发抓取坐标系G之间的变换关系GTEg的配置文件
        gte_config_path = rospy.get_param('~gte_config_path',os.path.join(current_path,'config/ur5/gripper/AG95/GTEg.tf'))
        #机械臂末端link与夹爪指尖固连坐标系E变换关系LTE的配置文件
        lte_config_path = rospy.get_param('~lte_config_path', os.path.join(current_path,'config/ur5/gripper/AG95/LTE.tf'))
        #机械臂配置参数文件地址
        arm_config_path = rospy.get_param('~arm_config_path', os.path.join(current_path,'config/ur5/armConfig.yaml'))
        #获得模式，默认测试模式
        mode = rospy.get_param('~mode', 'test')


        #先从从 yaml读取关于机械臂的配置参数,[({参数名，参数值},命名空间),(),...]
        #并上传ros服务器,以robot_name为命名空间
        arm_config_list=rosparam.load_file(arm_config_path)
        for params, _ in arm_config_list:
            rosparam.upload_params(robot_name,params)



        #再从参数服务器读取刚上传的yaml配置参数
        self.retreat = rospy.get_param(robot_name+'/retreat', '0.15')
        self.max_acceleration_scaling_factor = rospy.get_param(robot_name+'/max_acceleration_scaling_factor', '0.1')
        self.max_velocity_scaling_factor = rospy.get_param(robot_name+'/max_velocity_scaling_factor', '0.2')

        self.initial_joints = rospy.get_param(robot_name+'/initial_joints', '')

        self.working_joints = rospy.get_param(robot_name+'/working_joints', '')
        self.place_joints = rospy.get_param(robot_name+'/place_joints', '')



        #读取GTEg变换
        self.GTEg =  RigidTransform.load(gte_config_path) 
        #读取LTE变换 (夹爪指尖固连坐标系在机械臂末端连杆中的位姿，一般用于非机械臂自带的夹爪)
        self.LTE =  RigidTransform.load(lte_config_path) 


        #BTLg    最终抓取状态时，机械臂末端Link相对于基座的位姿
        self.grasp_pose_wrist3=Pose()
        #BTLp    预抓取状态时，机械臂末端Link相对于基座的位姿
        self.pre_grasp_pose_wrist3=Pose()


        #构建tf发布器
        self.tf_broadcaster=tf.TransformBroadcaster()

        #self.grasp_config=GraspConfig()

        #创建多用途的TF监听器
        self.tf_listener = tf.TransformListener()
        #变换关系正确读取的标志位
        get_transform=False
        #等待并获取抓取父坐标系与机械臂基座之间的tf变换关系
        while not get_transform:
            try:
                if mode=='test':
                    get_transform = True
                    rospy.loginfo("==================Test mode====================")
                else:
                    self.tf_listener.waitForTransform(grasp_father_frame,base_frame, rospy.Time(), rospy.Duration(5.0))  
                    #抓取父坐标系相对于机械臂基座坐标系的位姿
                    self.BTC_trans, self.BTC_quater = self.tf_listener.lookupTransform(base_frame, grasp_father_frame, rospy.Time(0))
                    #将trans转换成为ndarry
                    self.BTC_trans=np.array(self.BTC_trans)
                    #self.BTC_quater= np.array(self.BTC_quater)
                    self.BTC_rot = quaternion_matrix(self.BTC_quater)
                    #截取为3*3
                    self.BTC_rot = self.BTC_rot[0:3,0:3]
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
        self.robot_arm = moveit_commander.MoveGroupCommander(move_group_name)
        #创建机械手规划对象
        #self.panda_hand=moveit_commander.MoveGroupCommander('hand')
        #
        self.robot_arm.set_max_acceleration_scaling_factor(self.max_acceleration_scaling_factor)
        self.robot_arm.set_max_velocity_scaling_factor(self.max_velocity_scaling_factor)
        #通过此发布器发布规划的轨迹
        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                               DisplayTrajectory,
                                               queue_size=20)
        # 获取末端执行器名称
        self.end_effector_link = self.robot_arm.get_end_effector_link()
        rospy.loginfo("End effector detected {}".format(self.end_effector_link))         

        # 设置允许机械臂末位姿的错误余量
        self.robot_arm.set_goal_position_tolerance(0.01)#1cm
        self.robot_arm.set_goal_orientation_tolerance(0.05)#

        #不允许规划失败重规划,规划时间只允许5秒钟,否则很浪费时间
        self.robot_arm.allow_replanning(False)
        self.robot_arm.set_planning_time(5)
        
        #移动到工作姿态
        self.move_to_joints(self.robot_arm,self.working_joints,tag="working pose")
        #张开夹爪
        #self.set_gripper(0.078,epsilon=0.01)#张开8cm
        rospy.set_param("/robot_state", "ready")
        rospy.loginfo("Ready to grasp, working pose")

        ######################开始等待接收夹爪姿态#########################
        rospy.loginfo("Waiting for gripper pose")
        self.callback_done=False
        #刚开始认为抓取数量为0
        self.No_grasp = True

        if mode=='test':#测试模式
            #pass
            self.grasp_test()
        else:
            rospy.Subscriber(grasp_list_topic, GraspConfigList, self.Callback,queue_size=1)

        #######################执行抓取####################################
        while not rospy.is_shutdown():

            #判断回调函数是否处理完
            if self.callback_done:
                self.callback_done=False
                #处理完发现抓取数量为0
                if self.No_grasp:
                    rospy.sleep(1)
                    rospy.loginfo("Get No Grasp !")
                    continue
                else:
                    #如果抓取数量不是0
                    self.No_grasp =True
            else:
                rospy.sleep(0.5)
                continue

            

            #移动至预抓取姿态
            rospy.set_param("/robot_state", "moving")
            rospy.loginfo('Move to pre_grasp pose')
            self.robot_arm.set_start_state_to_current_state()  #以当前姿态作为规划起始点
            success=self.robot_arm.go(self.pre_grasp_pose_wrist3,wait=True)
            self.robot_arm.stop()
            self.robot_arm.clear_pose_targets()
            
            if not success:
                rospy.loginfo('Failed to move to pre_grasp pose!')
                rospy.sleep(1)
                rospy.set_param("/robot_state", "ready")
                continue

            rospy.loginfo('Succeed')

            rospy.sleep(1)#等待机械臂稳定


            #再设置当前姿态为起始姿态
            self.robot_arm.set_start_state_to_current_state()  
            #
            waypoints = []
            wpose=self.robot_arm.get_current_pose().pose
            wpose.position.x=  self.grasp_pose_wrist3.position.x
            wpose.position.y=  self.grasp_pose_wrist3.position.y
            wpose.position.z=  self.grasp_pose_wrist3.position.z
            waypoints.append(copy.deepcopy(wpose))

            #规划从当前位姿，保持姿态，转移到目标夹爪姿态的路径
            (plan, fraction) = self.robot_arm.compute_cartesian_path(
                waypoints,   # waypoints to follow
                0.01,        # eef_step
                0.0)         # jump_threshold
             ##显示轨迹
            display_trajectory = DisplayTrajectory()
            display_trajectory.trajectory_start = self.robot_arm.get_current_state()
            display_trajectory.trajectory.append(plan)
            display_trajectory_publisher.publish(display_trajectory)

            #执行,并等待这个轨迹执行成功
            new_plan=self.scale_trajectory_speed(plan,0.2)
            self.robot_arm.execute(new_plan,wait=True)

            #执行抓取
            rospy.loginfo("Start to grasp")
            #self.set_gripper(0.01,epsilon=0.4)#张开3cm
            rospy.sleep(1)

            ####################返回到预备抓取状态####################
            waypoints = []
            wpose=self.robot_arm.get_current_pose().pose
            wpose.position.x=  self.pre_grasp_pose_wrist3.position.x
            wpose.position.y=  self.pre_grasp_pose_wrist3.position.y
            wpose.position.z=  self.pre_grasp_pose_wrist3.position.z
            waypoints.append(copy.deepcopy(wpose))
            
            #规划从当前位姿，保持姿态，转移到目标夹爪姿态的路径
            (plan, fraction) = self.robot_arm.compute_cartesian_path(
                waypoints,   # waypoints to follow
                0.01,        # eef_step
                0.0)         # jump_threshold
            #显示轨迹
            display_trajectory = DisplayTrajectory()
            display_trajectory.trajectory_start = self.robot_arm.get_current_state()
            display_trajectory.trajectory.append(plan)
            display_trajectory_publisher.publish(display_trajectory)
            #执行,并等待后撤成功
            new_plan=self.scale_trajectory_speed(plan,0.6)
            self.robot_arm.execute(new_plan,wait=True)
            ######################移动到工作状态############################
            self.move_to_joints(self.robot_arm,self.working_joints,tag="working pose")
            #self.set_gripper(0.08,epsilon=0.4)#张开8cm
            rospy.loginfo("Working  pose")

            #########################移动放到放置状态#######################
            self.move_to_joints(self.robot_arm,self.place_joints,tag="place pose")
            #self.set_gripper(0.08,epsilon=0.4)#张开8cm
            rospy.loginfo("place  pose")
            rospy.sleep(1)
            ######################移动到工作状态############################
            self.move_to_joints(self.robot_arm,self.working_joints,tag="working pose")
            #self.set_gripper(0.08,epsilon=0.4)#张开8cm
            rospy.set_param("/robot_state", "ready")
            rospy.loginfo("Ready to grasp, working pose")
            rospy.sleep(2)

            if mode=='test':#测试模式
                self.callback_done=True
                self.No_grasp = False
                rospy.set_param("/robot_state", "moving")
                self.move_to_joints(self.robot_arm,self.working_joints,tag="working pose")
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

    
    def Callback(self,grasp_config_list): 
        """
        分别计算:
        机械臂末端link坐标系相对于base_link坐标系下的预抓取姿态  BTLp
        机械臂末端link坐标系相对于base_link坐标系下的最终抓取姿态  BTLg
        """

        #如果机械臂正在移动，不再对接收到的抓取进行处理
        if  rospy.get_param("/robot_state")=='moving':
            #标志回调函数处理完毕
            self.callback_done=True
            return 0
        #如果处于预备状态，判断话题中合法抓取数量
        if len(grasp_config_list.grasps)==0:
            self.callback_done=True
            return 0
        else:
            #话题中给定抓取数量不为0
            self.No_grasp =False
        rospy.loginfo('Get grasp num  {}'.format(len(grasp_config_list.grasps)))


        #认为抓取列表中的第一个抓取是最优抓取，把它读出来
        best_grasp_config = grasp_config_list.grasps[0]
        #将最优抓取构建为矩阵形式，即CTG
        CTG_trans = np.array([best_grasp_config.position.x, 
            best_grasp_config.position.y, best_grasp_config.position.z])

        CTG_approach = np.array([best_grasp_config.approach.x,
            best_grasp_config.approach.y,best_grasp_config.approach.z])
        CTG_binormal = np.array([best_grasp_config.binormal.x,
            best_grasp_config.binormal.y,best_grasp_config.binormal.z])
        CTG_axis = np.array([best_grasp_config.axis.x,
            best_grasp_config.axis.y,best_grasp_config.axis.z])
        
        CTG_rot = np.c_[CTG_approach,np.c_[CTG_binormal,CTG_axis]].reshape(3,3).swapaxes(0,1)



        #计算得到抓取坐标系G在基座坐标系B中的位姿
        BTG_rot = self.BTC_rot.dot(CTG_rot)
        BTG_trans = self.BTC_rot.dot(CTG_trans)+self.BTC_trans

        #读取 夹爪**指尖**固连坐标系E与抓取坐标系G之间的关系 GTEg
        GTEg_trans = self.GTEg.translation #
        GTEg_rot = self.GTEg.rotation #3*3

        #抓取情形下，夹爪指尖坐标系E 相对于基座的位姿
        BTEg_rot = BTG_rot.dot(GTEg_rot)
        BTEg_trans = BTG_rot.dot(GTEg_trans)+BTG_trans
        BTEg_rot_ = np.eye(4)
        BTEg_rot_[:3,:3] = BTEg_rot
        #用于显示
        BTEg_quater = quaternion_from_matrix(BTEg_rot_)

        #预备抓取情形下，夹爪指尖坐标系E 相对于基座的位姿
        BTEp_rot = BTEg_rot
        BTEp_trans = BTEg_trans - BTEg_rot[:,2]*self.retreat #[3,]
        #用于显示
        BTEp_quater = BTEg_quater

        #LTE      夹爪指尖固连坐标系与机械臂末端Link的变换关系
        LTE_trans=self.LTE.translation#偏移
        LTE_rot = self.LTE.rotation#[3,3]
        #ETL
        ETL_rot = LTE_rot.T
        ETL_trans = - ETL_rot.dot(LTE_trans)


        #抓取情形下，末端连杆坐标系L 相对基座坐标系的位姿
        BTLg_rot = BTEg_rot.dot(ETL_rot)
        BTLg_trans = BTEg_rot.dot(ETL_trans)+BTEg_trans

        #预备抓取情形下，末端连杆坐标系L 相对基座坐标系的位姿
        BTLp_rot = BTEp_rot.dot(ETL_rot)
        BTLp_trans = BTEp_rot.dot(ETL_trans)+BTLp_trans
        BTLg_rot_ = np.eye(4)
        BTLg_rot_[:3,:3] = BTLg_rot
        BTLg_quater=quaternion_from_matrix(BTLg_rot_)


        #BTLg    转换为GraspConfig形式
        self.grasp_pose_wrist3.position.x = BTLg_trans[0]
        self.grasp_pose_wrist3.position.y = BTLg_trans[1]
        self.grasp_pose_wrist3.position.z = BTLg_trans[2]
        self.grasp_pose_wrist3.orientation.x = BTLg_quater[0]
        self.grasp_pose_wrist3.orientation.y = BTLg_quater[1]
        self.grasp_pose_wrist3.orientation.z = BTLg_quater[2]
        self.grasp_pose_wrist3.orientation.w = BTLg_quater[3]

        #BTLp    拷贝 BTLg  仅仅修改位置参数即可
        self.pre_grasp_pose_wrist3 = copy.deepcopy(self.grasp_pose_wrist3)
        self.pre_grasp_pose_wrist3.position.x = BTLp_trans[0]
        self.pre_grasp_pose_wrist3.position.y = BTLp_trans[1]
        self.pre_grasp_pose_wrist3.position.z = BTLp_trans[2]

        #发布BTEg    最终抓取状态时，AG95相对于基座的位姿
        self.tf_broadcaster.sendTransform(
            BTEg_trans,
            BTEg_quater,
            rospy.Time.now(),
            "BTEg",
            "base_link")        
        #发布BTEp    预抓取状态时，AG95相对于基座的位姿
        self.tf_broadcaster.sendTransform(
            BTEp_trans,
            BTEp_quater,   #与抓取情形下相同
            rospy.Time.now(),
            "BTEp",
            "base_link")   

        #标志回调函数处理完毕
        self.callback_done=True 

    def grasp_test(self): 
        """
        给定AG95坐标系在base坐标系下的最终抓取姿态  BTEg
        分别计算:
        wrist_3_link坐标系相对于base_link坐标系下的预抓取姿态  BTLp
        wrist_3_link坐标系相对于base_link坐标系下的最终抓取姿态  BTLg
        """

        #BTEg    最终抓取状态时，AG95相对于基座的位姿
        BTEg_trans =np.array([0.58166,0.08053,0.11798])
        BTEg_quater=np.array([-0.69264,0.72096,-0.0053162,-0.020851])#将姿态转换为四元数形式
        BTEg_rot = quaternion_matrix(BTEg_quater)
        BTEg_rot = BTEg_rot[:3,:3]


        #LTE      夹爪指尖固连坐标系与机械臂末端Link的变换关系
        LTE_trans=self.LTE.translation#偏移
        LTE_rot = self.LTE.rotation#[3,3]

        #BTEp    预抓取状态时，AG95相对于基座的位姿
        BTEp_trans = BTEg_trans - BTEg_rot[:,2]*self.retreat #[3,]
        BTEp_rot = BTEg_rot 

        #BTLg    最终抓取状态时，机械臂末端link相对于基座的位姿
        BTLg_rot = BTEg_rot.dot(LTE_rot.T)#姿态[3,3]
        BTLg_trans = BTEg_trans - BTLg_rot.dot(LTE_trans)#位置
        BTLg_rot_ = np.eye(4)
        BTLg_rot_[:3,:3] = BTLg_rot
        BTLg_quater=quaternion_from_matrix(BTLg_rot_)

        self.grasp_pose_wrist3.position.x = BTLg_trans[0]
        self.grasp_pose_wrist3.position.y = BTLg_trans[1]
        self.grasp_pose_wrist3.position.z = BTLg_trans[2]
        self.grasp_pose_wrist3.orientation.x = BTLg_quater[0]
        self.grasp_pose_wrist3.orientation.y = BTLg_quater[1]
        self.grasp_pose_wrist3.orientation.z = BTLg_quater[2]
        self.grasp_pose_wrist3.orientation.w = BTLg_quater[3]

        #BTLp    预抓取状态时，机械臂末端link相对于基座的位姿
        self.pre_grasp_pose_wrist3 = copy.deepcopy(self.grasp_pose_wrist3)

        BTLp_rot = BTLg_rot
        BTLp_trans = BTEp_trans - BTLp_rot.dot(LTE_trans)
        self.pre_grasp_pose_wrist3.position.x = BTLp_trans[0]
        self.pre_grasp_pose_wrist3.position.y = BTLp_trans[1]
        self.pre_grasp_pose_wrist3.position.z = BTLp_trans[2]


        #发布BTEg    最终抓取状态时，AG95相对于基座的位姿
        self.tf_broadcaster.sendTransform(
            BTEg_trans,
            BTEg_quater,
            rospy.Time.now(),
            "BTEg",
            "base_link")        
        #发布BTEp    预抓取状态时，AG95相对于基座的位姿
        self.tf_broadcaster.sendTransform(
            BTEp_trans,
            BTEg_quater,   #与抓取情况下相同
            rospy.Time.now(),
            "BTEp",
            "base_link")   

        #标志回调函数处理完毕
        self.callback_done=True 
        self.No_grasp = False


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
        GoGrasp()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Arm tracker node terminated.")

    
    