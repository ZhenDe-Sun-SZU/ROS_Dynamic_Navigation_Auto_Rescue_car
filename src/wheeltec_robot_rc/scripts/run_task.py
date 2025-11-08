# #!/usr/bin/python3

# import sys
# import os
# import yaml
# import _thread
# import threading
# import pickle

# import rospy
# import rospkg
# import actionlib
# from actionlib_msgs.msg import *
# from move_base_msgs.msg import MoveBaseActionResult, MoveBaseResult
# # import common.msg
# # import common.srv
# # from common.msg import MoveStraightDistanceAction, TurnBodyDegreeAction
# # import common.action
# from std_msgs.msg import String




# class RobotMoveAction:
#     def __init__(self):
#         # # 创建控制spark直走的action客户端
#         # self.move_action_cli = actionlib.SimpleActionClient(
#         #     'move_straight', MoveStraightDistanceAction)
#         # self.move_action_cli.wait_for_server(
#         #     timeout=rospy.Duration.from_sec(3))

#         # # 创建控制spark旋转的action客户端
#         # self.turn_action_cli = actionlib.SimpleActionClient(
#         #     'turn_body', TurnBodyDegreeAction)
#         # self.turn_action_cli.wait_for_server(
#         #     timeout=rospy.Duration.from_sec(3))


#         # # 创建获取spark前后距离的service客户端
#         # rospy.wait_for_service('/get_distance')
#         # self.distance_srv = rospy.ServiceProxy(
#         #     'get_distance', common.srv.GetFrontBackDistance)

#         # 创建导航地点的话题发布者
#         self.goto_local_pub = rospy.Publisher(
#             "mark_nav", String, queue_size=1)

#     def goto_local(self, name):
#         '''
#         根据目标点名称,发布目标位置到MoveBase服务器,根据返回状态进行判断
#         @return: True 为成功到达, False 为失败
#         '''

#         # 发布目标位置
#         self.goto_local_pub.publish("go "+name)

#         # 设定1分钟的时间限制，进行阻塞等待
#         try:
#             ret_status = rospy.wait_for_message(
#                 'move_base/result', MoveBaseActionResult, rospy.Duration(60)).status.status
#         except Exception:
#             rospy.logwarn("nav timeout!!!")
#             ret_status = GoalStatus.ABORTED

#         # 如果一分钟之内没有到达，放弃目标
#         if ret_status != GoalStatus.SUCCEEDED:
#             rospy.Publisher("move_base/cancel", GoalID, queue_size=1).publish(
#                 GoalID(stamp=rospy.Time.from_sec(0.0), id=""))
#             try:
#                 rospy.wait_for_message(
#                     'move_base/result', MoveBaseActionResult, rospy.Duration(3))
#             except Exception:
#                 rospy.logwarn("move_base result timeout. this is abnormal.")
#             rospy.loginfo("==========Timed out achieving goal==========")
#             return False
#         else:
#             rospy.loginfo("==========Goal succeeded==========")
#             return True
    


# class AutoAction:
#     def __init__(self):
#         # # 初始化节点
#         # if init_node:
#         rospy.init_node('spark_auto_match_node', anonymous=True)

#         print("========ready to task===== ")

#         # 实例化Robot
#         try: self.robot = RobotMoveAction()
#         except Exception as e:  print("except robot:",e)
#         print("========实例化Robot===== ")

#         # 订阅任务控制指令的话题
#         self.task_cmd_sub = rospy.Subscriber("/task_start_flag", String, self.task_cmd_cb) # 订阅任务开始与否信号
#         self.task_run_th = threading.Thread(target=lambda: "pass") # 创建线程对象
#         self.stop_flag = False  # 任务的启停标志
#         rospy.loginfo("spark_auto_match_node is ready")


#     def run_task(self):
#         ret = False # 是否导航成功标志
#         item_type = 0 

#         # ===== 现在开始执行任务 =====
#         rospy.loginfo("start task now.")

#         if self.stop_flag: return


#         # ===== 导航到分类区 =====
#         if self.robot.goto_local("start"):
#             rospy.sleep(2)
#         else :
#             rospy.logerr("Navigation to Classification_area failed,please run task again ")
#             self.stop_flag = True


#         rospy.logwarn("***** task finished *****")
#         rospy.logwarn("if you want to run task again. ")
#         rospy.logwarn("Re-send a message to hm_task_cmd topic. ")
#         rospy.logwarn("Or press Ctrl+C to exit the program")

#     def task_cmd_cb(self,flag):
#         if flag :
#             if not self.task_run_th.is_alive():
#                 self.stop_flag = False
#                 self.task_run_th = threading.Thread(target=self.run_task, args=())
#                 self.task_run_th.start()
#                 rospy.loginfo("start task!!!")
#             else:
#                 rospy.logwarn("waiting for thread exit...")
#                 self.stop_flag = True
#                 self.task_run_th.join()
#                 rospy.logwarn("thread exit success")



# if __name__ == '__main__':
#     try:
#         AutoAction()
#         rospy.spin()
#     except rospy.ROSInterruptException:
#         print("Mark_move finished.")


#!/usr/bin/env python3
# -*- coding: UTF-8 -*-

import rospy
import actionlib
from actionlib_msgs.msg import *
from std_msgs.msg import String
from geometry_msgs.msg import Pose, Point, Quaternion, Twist, PointStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import tf
import tf.transformations as tf_transformations
from visualization_msgs.msg import Marker
from math import radians, pi
import numpy as np
import yaml
import rospkg
import os
import pickle

class MarkNav():
    def __init__(self):
        # 初始化节点
        rospy.init_node('MarkNav')
        # 发布TWist消息控制机器人
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        # 订阅move_base服务器的消息
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        # 等待move_base服务器建立
        self.move_base.wait_for_server(rospy.Duration(20))
        print('ready')
        # 订阅标记事件
        rospy.Subscriber('mark_nav', String, self.mark_nav)
        # 定义标记地点字典
        global dict_mark
        dict_mark = {}
        # 监听TF坐标
        self.listener = tf.TransformListener()
        rospy.sleep(1) # need to delay
    
    def mark_nav(self,mark_name):
        '''
        设定标定地点，与后面导航相关，采用字典的方式存放
        建议标记地点名字,分类区、分拣区、收取区
        '''

        rospack = rospkg.RosPack() 
        wheeltec_path = rospack.get_path('wheeltec_robot')
        mark_map_path = os.path.join(wheeltec_path,'src', 'turn_on_wheeltec_robot', 'map', 'WHEELTEC.pgm')  # 标记地图yaml文件地址
        

        # 转化成字符型变量
        mark_name = str(mark_name)
        # 结束地点标记
        if str.find(mark_name,str("finish"))!= -1:
            # 将字典信息写入 pkl 文件
            with open(mark_map_path, 'wb') as file:
                pickle.dump(dict_mark, file)
            print("已保存pkl文件,文件保存在:",mark_map_path)
            print("当前字典内容：\n",dict_mark)


        # 前往地点    
        if str.find(mark_name,str("go"))!= -1:
            # 从文件中加载数据
            with open(mark_map_path, 'rb') as file:
                self.mark_map = pickle.load(file)
            # 输出加载的数据
            print("mark_dict=",self.mark_map)
            # 提取mark名称
            mark_name = (mark_name.split())[2]
            # 判断地点位置是否在地点字典中
            if mark_name in self.mark_map:
                self.navigation(mark_name)
            else:
                print("此地点尚未登陆在字典中")


        # 学习地点
        if str.find(mark_name,str("learn"))!= -1:
            # 提取mark名称
            mark_name = (mark_name.split())[2]
            # 获取当前位置
            self.currrent_position = self.get_currect_pose()
            # 标记当前位置
            dict_mark[mark_name] = self.currrent_position
            print("当前字典内容：\n",dict_mark)

    # 获取当前位置
    def get_currect_pose(self):
        (target_trans, target_rot) = self.listener.lookupTransform("map", "base_footprint", rospy.Time(0))
        print("target_trans:",target_trans)
        print("target_rot:",target_rot)
        return tf_transformations.compose_matrix(translate=target_trans, angles=tf_transformations.euler_from_quaternion(target_rot))

    def navigation(self,mark_name):
        '''
        根据地点进行导航
        '''
        print("start navigation")
        # movebase初始化
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        # 设定目标地点
        self.drop_position = self.mark_map[mark_name]
        tran = tf_transformations.translation_from_matrix(self.drop_position)
        quat = tf_transformations.quaternion_from_matrix(self.drop_position)
        pose = Pose(Point(tran[0], tran[1], tran[2]), Quaternion(quat[0], quat[1], quat[2], quat[3]))
        goal.target_pose.pose = pose
        # 把目标位置发送给MoveBaseAction的服务器
        self.move_base.send_goal(goal)

if __name__ == '__main__':
    try:
        MarkNav()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Mark_move finished.")