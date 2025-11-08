import rospy
from xy_package.msg import PointMessage 

import cv2
from ultralytics import YOLO

def talker():
    pub = rospy.Publisher('points_chatter',XXXXX,queue_size=10)
    rospy.init_node('points_talker',anonymous=True)
    rate =rospy.Rate(1)
    while not rospy.is_shutdown():
        point_msg = PointMessage()
        point_msg.x1 = 1#test
        point_msg.y1 = 1
        point_msg.x2 = 2
        point_msg.y2 = 2
        pub.publish(point_msg)
        rate.sleep()

# Load the YOLO model
k=0
b=0
model = YOLO("yolo11n-pose.pt")

# Open the video file
cap = cv2.VideoCapture(0)

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
            b=r.boxes.xyxy.numpy()
                # Display the annotated frame
            print(f"boxes:{b},keypoint:{k},{type(b)}{type(k)}")
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


if __name__=='__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass







