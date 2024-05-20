import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import torch
import threading
import queue
import time

# Load YOLOv5 model
model = torch.hub.load('ultralytics/yolov5:v7.0', 'yolov5s')  # Use the latest YOLOv5 version
model.eval()

# Initialize CvBridge
bridge = CvBridge()

# Flag for video capture loop
exit_flag = False

# Class indices for humans and hands (adjust based on your YOLOv5 class indices)
human_class_index = 0  # Replace with the correct class index for humans
hand_class_index = 1  # Replace with the correct class index for hands

# Queue for passing images between threads
image_queue = queue.Queue()

# Variable for frame skipping
frame_skip = 3  # Adjust the frame skip value as needed

# Dictionary to store tracking information for each person
person_dict = {}

def image_callback(msg):
    global exit_flag

    # Convert ROS Image message to OpenCV format
    cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    # Resize input image (adjust dimensions as needed)
    resized_image = cv2.resize(cv_image, (640, 480))  # Example dimensions

    image_queue.put(resized_image)

# def is_hand_gesture(person_bbox, hands_bboxes):
#     # You can implement logic to check if a person is showing a hand gesture based on bounding boxes
#     # For example, you may check if any hand bounding box is within the person bounding box
#     person_x, person_y, person_w, person_h = person_bbox
#     for hand_bbox in hands_bboxes:
#         hand_x, hand_y, hand_w, hand_h = hand_bbox
#         if person_x < hand_x < person_x + person_w and person_y < hand_y < person_y + person_h:
#             return True
#     return False

def inference():
    global exit_flag, model, human_class_index, frame_skip

    frame_count = 0

    while not exit_flag:
        if not image_queue.empty():
            frame = image_queue.get()

            frame_count += 1

            # Skip frames if needed
            if frame_count % frame_skip != 0:
                continue

            # Run YOLOv5 inference on the frame
            results = model(frame)

            # Check if there are any detections
            if results.xyxy is not None and len(results.xyxy) > 0:
                # Extract bounding box coordinates for humans
                human_bboxes = results.xyxy[0].cpu().numpy()
                # Filter out only the human detections
                human_bboxes = human_bboxes[human_bboxes[:, 5] == human_class_index]

                # Segment human regions and create binary masks
                for idx, bbox in enumerate(human_bboxes):
                    x1, y1, x2, y2, _, _ = bbox.astype(int)
                    segmented_human = frame[y1:y2, x1:x2]

                    # Convert segmented human image to grayscale
                    gray_image = cv2.cvtColor(segmented_human, cv2.COLOR_BGR2GRAY)

                    # Apply threshold to obtain binary segmented image
                    _, binary_segmented_human = cv2.threshold(gray_image, 0, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)

                    # Process the binary segmented human image (e.g., save, display, further analysis)
                    cv2.imshow(f"Binary Segmented Human {idx}", binary_segmented_human)
                    cv2.waitKey(1)

            # Display the processed image with bounding boxes
            cv2.imshow("RGB Image", frame)
            cv2.waitKey(1)



def launch_kinect_and_capture():
    global exit_flag

    # Subscribe to the Kinect RGB image topic
    rospy.init_node("kinect_rgb_subscriber", anonymous=True)
    rospy.Subscriber("/camera/rgb/image_color", Image, image_callback)

    # Start the inference thread
    inference_thread = threading.Thread(target=inference)
    inference_thread.start()

    # Wait for 'q' key press to exit the video
    while not exit_flag and not rospy.is_shutdown():
        rospy.sleep(0.05)  # Adjust the sleep duration based on your desired FPS

    # Set exit flag for the inference thread to stop
    exit_flag = True

    # Wait for the inference thread to finish
    inference_thread.join()

    # Release any remaining resources
    cv2.destroyAllWindows()

if __name__ == "__main__":
    # Launch Kinect RGB camera and capture
    launch_kinect_and_capture()
