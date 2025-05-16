#!/usr/bin/env python3
import rospy
import torch
import numpy as np
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_srvs.srv import SetBool, SetBoolResponse
import warnings

warnings.filterwarnings("ignore", category=FutureWarning)


class HumanFollower:
    def __init__(self):
        rospy.init_node('human_follower')

        # Set up camera and control interfaces
        self.bridge = CvBridge()
        self.rgb_image = None
        self.depth_image = None
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.is_active = True

        # YOLOv5 model for person detection (class 0)
        # self.model = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True)
        self.model = torch.hub.load('ultralytics/yolov5', 'yolov5n', pretrained=True)
        self.model.classes = [0]

        # Command rate limiter
        self.last_cmd_time = rospy.Time.now()
        self.cmd_interval = rospy.Duration(1) # 1 second 

        # Subscribers and services
        rospy.Subscriber('/camera/color/image_raw', Image, self.rgb_callback, queue_size=1)
        rospy.Subscriber('/camera/aligned_depth_to_color/image_raw', Image, self.depth_callback, queue_size=1)
        rospy.Service('toggle_follower', SetBool, self.handle_toggle)

        rospy.spin()

    def handle_toggle(self, req):
        self.is_active = req.data
        msg = "Human follower activated" if self.is_active else "Human follower stopped"
        return SetBoolResponse(success=True, message=msg)
    
    def get_depth_value(self, foot_x, foot_y, h, w):
        # Check if depth image is available
        if self.depth_image is None:
            rospy.logwarn("Depth image not available")
            return None
        
        # Boundaries for a 3x3 patch
        x0 = max(0, foot_x - 1)
        x1 = min(w, foot_x + 2)
        y0 = max(0, foot_y - 1)
        y1 = min(h, foot_y + 2)
    
        patch = self.depth_image[y0:y1, x0:x1]
        valid = patch[(patch > 0) & (~np.isnan(patch))]
    
        return np.min(valid) / 1000.0 if valid.size > 0 else None


    def get_offset(self, foot_x, center_x):
        return foot_x - center_x


    def get_foot_position(self, x1, y1, x2, y2):
        foot_x = int((x1 + x2) / 2)
        foot_y = int(y1 + 0.95 * (y2 - y1))  # 0.05 from bottom
        return foot_x, foot_y
    
    def process_images(self):
        if not self.is_active:
            return
        
        twist = Twist()
        height, width, _ = self.rgb_image.shape

        # Draw a vertical line in the center of the image
        center_x = width // 2
        cv2.line(self.rgb_image, (center_x, 0), (center_x, height), (0, 255, 0), 2)

        boxes = self.model(self.rgb_image).xyxy[0].cpu().numpy()
        for box in boxes:
            x1, y1, x2, y2, conf, cls = box

            # Check if the detected object is a person (class 0) with sufficient confidence
            if conf < 0.2 or cls != 0:
                rospy.loginfo(f"Skipping detection: Class {cls}, Confidence {conf:.2f}")
                continue

            foot_x, foot_y = self.get_foot_position(x1, y1, x2, y2)

            if foot_x < 0 or foot_x >= width or foot_y < 0 or foot_y >= height:
                rospy.logwarn("Foot position out of bounds")
                continue

            self.draw_overlay(x1, y1, x2, y2, foot_x, foot_y, conf)


            depth_value = self.get_depth_value(foot_x, foot_y, height, width)
            if depth_value is None:
                depth_value = 1.0 # Default to 1.0 if depth is invalid, to avoid stopping angular velocity

            offset = self.get_offset(foot_x, width // 2)
            norm_offset = offset / (width / 2.0)

            # Enforce rate limit
            now = rospy.Time.now()
            if (now - self.last_cmd_time) < self.cmd_interval:
                rospy.loginfo(f"Rate limit: only {(now - self.last_cmd_time).to_sec():.2f}s since last command")
                break

            twist = self.compute_cmd_vel(norm_offset, depth_value)
            self.last_cmd_time = rospy.Time.now()
            
            rospy.loginfo(f"Offset: {offset}, Depth: {depth_value:.2f} → "
                          f"Linear: {twist.linear.x:.2f}, Angular: {twist.angular.z:.2f}")
            
            
            break  # only act on the first person

        self.cmd_pub.publish(twist)
        self.display_image()



    def rgb_callback(self, msg):
        img_time = msg.header.stamp.to_sec()
        # rospy.loginfo(f"[RGB] Image timestamp: {img_time:.6f}, Current time: {rospy.Time.now().to_sec():.6f}")
        self.rgb_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.process_images()
        self.rgb_image = None  # Reset after processing
        self.depth_image = None  # Reset depth image to avoid stale data


    def depth_callback(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        return
    
    def compute_cmd_vel(self, norm_offset, depth):
        twist = Twist()
    
        # 1. Set linear speed based on distance
        if depth > 3.0:
            twist.linear.x = 0.3  # Far → move fast
        elif depth > 2.0:
            twist.linear.x = 0.25  # Medium distance → move slower
        elif depth > 1.1:
            twist.linear.x = 0.2  # Normal follow
        else:
            twist.linear.x = 0.0   # Stop
    
        # 2. Get base angular speed
        base_angular = self.compute_angular_from_offset(norm_offset)
    

        twist.angular.z = base_angular
    
        return twist


    def compute_angular_from_offset(self, norm_offset):
        abs_offset = abs(norm_offset)

        if norm_offset > 0:  # Offset to the left → turn right
            if abs_offset < 0.3:
                angular = 0.0
            elif abs_offset < 0.5:
                angular = -0.20  
            elif abs_offset < 0.8:
                angular = -0.25
            else:
                angular = -0.35
        else:  # Offset to the right → turn left
            if abs_offset < 0.6: # too frequent command seems will be failed
                angular = 0.0
            else:
                angular = 0.05

        return angular



    def draw_overlay(self, x1, y1, x2, y2, foot_x, foot_y, conf):
        cv2.rectangle(self.rgb_image, (int(x1), int(y1)), (int(x2), int(y2)), (255, 0, 0), 2)
        cv2.circle(self.rgb_image, (foot_x, foot_y), 5, (0, 0, 255), -1)
        text_y = max(int(y1) - 10, 10)
        cv2.putText(self.rgb_image, f"{conf:.2f}", (int(x1), text_y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

    def display_image(self):
        resized = cv2.resize(self.rgb_image, None, fx=2.0, fy=2.0)
        cv2.imshow("Human Follower View", resized)
        cv2.waitKey(1)


if __name__ == '__main__':
    try:
        HumanFollower()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()
