#!/home/wheeltec/miniforge3/envs/pose/bin/python
#coding=utf-8
import cv2
from ultralytics import YOLO
import rospy
import numpy as np
from geometry_msgs.msg import Twist
from wheeltec_robot_rc.msg import pointss

if __name__ == "__main__":

    # 2.初始化 ROS 节点
    rospy.init_node("pose_publish")
    # 3.创建发布者对象
    pub = rospy.Publisher("points_chatter",pointss,queue_size=1000)
    msg = pointss()

    # Load the YOLO model
    model = YOLO("yolo11n-pose.pt")

    # Open the video file
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FPS,5)
    X_1=0
    Y_1=0
    X_2=0
    Y_2=0

    # Loop through the video frames
    while cap.isOpened():
        # Read a frame from the video
        success, frame = cap.read()

        if success:
            # Run YOLO inference on the frame
            results = model(frame)
            # Visualize the results on the frame
            annotated_frame = results[0].plot()
            for r in results:
                k=r.keypoints.xy.numpy()
                #t=k[0,1]
                b=r.boxes.xyxy.numpy()
                    # Display the annotated frame
                print(f"boxes:{b},keypoint:{k},{type(b)}{type(k)}")
                # 提取keypoints列表中的倒数第三行和倒数第四行的数据
                if not np.all(np.equal(k, 0)):
                    X_1=k[0,13,0]
                    Y_1=k[0,13,1]
                    X_2=k[0,14,0]
                    Y_2=k[0,14,1]
                # 将提取的数据填充到ROS消息中
                msg.x1 = X_1 # 假设x坐标是列表的第一个元素
                msg.x2 = X_2 # 同上
                msg.y1 = Y_1 # 假设y坐标是列表的第二个元素
                msg.y2 = Y_2# 同上
                #print("Received x1:",msg.x1)

                    # 发布消息
                pub.publish(msg)
            cv2.imshow("YOLO Inference", annotated_frame)
            # Break the loop if 'q' is pressed
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
        else:
            # Break the loop if the end of the video is reached
            break
    # Release the video capture object and close the display window
    cap.release()
    cv2.destroyAllWindows()


