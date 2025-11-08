#!/home/wheeltec/miniforge3/envs/pose/bin/python3.9
#coding=utf-8
import cv2
import numpy as np
from ultralytics import YOLO
import rospy
from geometry_msgs.msg import Twist
from wheeltec_robot_rc.msg import pointss
from sensor_msgs.msg import Image
# from cv_bridge import CvBridge
from core import CvBridge,CvBridgeError
cv_image=None
new_image=False


def callback(msg):
    global cv_image,new_image
    bridge=CvBridge()
    try:
        cv_image=bridge.imgmsg_to_cv2(msg,"bgr8")
        new_image=True
    except Exception as e:
        rospy.logerr("erro:%s",e)
def cam_subscriber():
    global cv_image,new_image# 2.初始化 ROS 节点
    rospy.init_node("pose_publish",anonymous=True)
    # 3.创建发布者对象
    pub = rospy.Publisher("points_chatter",pointss,queue_size=1000)
    msg = pointss()
    # Load the YOLO model
    model = YOLO("yolo11n-pose.pt")
    # rospy.Subscriber('camera_image',Image,callback)
    rospy.Subscriber('/camera/rgb/image_raw',Image,callback)
    rospy.loginfo("start")
    while not rospy.is_shutdown():
        if new_image:
            # Run YOLO inference on the frame
            results = model(cv_image)
            # Visualize the results on the frame
            annotated_frame = results[0].plot()
            for r in results:
                k=r.keypoints.xy.numpy()
                b=r.boxes.xyxy.numpy()
                    # Display the annotated frame
                print(f"boxes:{b},keypoint:{k},{type(b)}{type(k)}")
                # 提取keypoints列表中的倒数第三行和倒数第四行的数据
                if len(k) >= 6: # 确保列表中至少有6个元素
                    last_third_point = k[-3] # 倒数第三行
                    second_last_point = k[-4] # 倒数第四行
                    # 将提取的数据填充到ROS消息中
                    msg.x1 = last_third_point[0] # 假设x坐标是列表的第一个元素
                    msg.x2 = second_last_point[0] # 同上
                    msg.y1 = last_third_point[1] # 假设y坐标是列表的第二个元素
                    msg.y2 = second_last_point[1] # 同上
                    # 发布消息
                    pub.publish(msg)
            cv2.imshow("YOLO Inference",annotated_frame)
            cv2.waitKey(30)
            new_image=False
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
        else: 
            cv2.waitKey(1)
            if 0xFF == ord("q"):
                break
    cv2.destroyAllWindows()
if __name__=='__main__':
    try:
        cam_subscriber()
    except rospy.ROSInterruptException:
        pass