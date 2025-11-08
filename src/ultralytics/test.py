import cv2
from ultralytics import YOLO

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






