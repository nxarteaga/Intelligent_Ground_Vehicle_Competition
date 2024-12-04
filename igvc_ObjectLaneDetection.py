import threading
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import cv2
import serial
import numpy as np
import webbrowser
import os
import requests

esp32_ip = "10.0.0.4"  # Default IP for ESP32 in SoftAP mode
port = 80

Direction_Wait = 0.80

# Commands to the ESP32 over Wi-Fi
commands = {
    "FORWARD": f"http://{esp32_ip}:{port}/FORWARD",
    "RIGHT": f"http://{esp32_ip}:{port}/CW",
    "LEFT": f"http://{esp32_ip}:{port}/CCW",
    "STOP": f"http://{esp32_ip}:{port}/STOP",
    "SLOW": f"http://{esp32_ip}:{port}/SLOW",
}

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback,
            10)
        self.subscription
        self.switching = False

    def listener_callback(self, msg: LaserScan):
        total_measurements = len(msg.ranges)
        resolution = 360 / total_measurements
        global Obstacle
        def degrees_to_index(deg):
            return int(deg / resolution)

        degree_ranges = (
            list(range(degrees_to_index(0), degrees_to_index(30))) +
            list(range(degrees_to_index(330), degrees_to_index(360)))
        )

        valid_ranges = [r for r in msg.ranges if msg.range_min < r < msg.range_max]
        threshold_max = 1.5
        threshold_min = 1.0

        majority = sum(1 for i in degree_ranges if i < len(valid_ranges) and threshold_min < valid_ranges[i] < threshold_max)

        if majority > 35:
            if not self.switching:
                print(f"Obstacle detected! Majority: {majority}.")
                #webbrowser.open(left_url)
                send_command("STOP")
                self.switching = True
                Obstacle = True
        else:
            if self.switching:
                print(f"Majority: {majority}. No obstacles detected.")
                #webbrowser.open(right_url)
                #send_command("RIGHT")
                if majority < 35:
                    self.switching = False
                    Obstacle = False
            #else:
                #print(f"Majority: {majority}. No obstacles detected.")
               # Obstacle = False

def kill_firefox():
    os.system('killall firefox')

def send_command(command):
    try:
        response = requests.get(commands[command])
        if response.status_code == 200:
            print(f"Command '{command}' sent successfully.")
        else:
            print(f"Failed to send command '{command}'. Status code: {response.status_code}")
    except requests.RequestException as e:
        print(f"Error sending command '{command}': {e}")

def convert_hsl(image):
    """Convert an image from RGB to HSL."""
    return cv2.cvtColor(image, cv2.COLOR_RGB2HLS)


def HSL_color_selection(image):
    """Apply color selection to the HSL images to blackout everything except for white lane lines."""
    converted_image = convert_hsl(image)
    lower_threshold = np.uint8([0, 200, 0])  # H, L, S
    upper_threshold = np.uint8([255, 255, 255])
    white_mask = cv2.inRange(converted_image, lower_threshold, upper_threshold)
    return cv2.bitwise_and(image, image, mask=white_mask)


def gray_scale(image):
    """Convert an image to grayscale."""
    return cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)


def gaussian_smoothing(image, kernel_size=5):
    """Apply Gaussian smoothing to the image."""
    return cv2.GaussianBlur(image, (kernel_size, kernel_size), 0)


def canny_detector(image, low_threshold=50, high_threshold=150):
    """Apply Canny edge detection to the image."""
    return cv2.Canny(image, low_threshold, high_threshold)


def region_selection(image):
    """Determine and cut the region of interest in the input image."""
    mask = np.zeros_like(image)
    rows, cols = image.shape[:2]
    vertices = np.array([[
        [cols * 0.01, rows * 0.80], #bottom left
        [cols * 0.15, rows * 0.65],  #top left
        [cols * 0.85, rows * 0.65],  #top right
        [cols * 0.99, rows * 0.80]  #bottom rightqqq
    ]], dtype=np.int32)
    cv2.fillPoly(mask, vertices, 255)
    return cv2.bitwise_and(image, mask)


def hough_transform(image):
    """Determine lines in the image using the Hough Transform."""
    rho = 1
    theta = np.pi / 180
    threshold = 50
    minLineLength = 7
    maxLineGap = 100
    lines = cv2.HoughLinesP(image, rho=rho, theta=theta, threshold=threshold,
                             minLineLength=minLineLength, maxLineGap=maxLineGap)
    return lines if lines is not None else []




def average_slope_intercept(lines, slope_thresh = 0.3):
    """Find the slope and intercept of the left and right lanes of each image."""
    if len(lines) == 0:  # Check if lines is empty
        return None, None  # Return None if no lines are detected

    left_lines = []  # (slope, intercept)
    left_weights = []  # (length,)
    right_lines = []  # (slope, intercept)
    right_weights = []  # (length,)
    
    for line in lines:
        for x1, y1, x2, y2 in line:
            if x1 == x2:  # Skip vertical lines
                continue
            slope = (y2 - y1) / (x2 - x1)

            # if abs(slope) < slope_thresh:
            #     continue

            intercept = y1 - (slope * x1)
            length = np.sqrt(((y2 - y1) ** 2) + ((x2 - x1) ** 2))

            if slope < 0:
                left_lines.append((slope, intercept))
                left_weights.append(length)
            else:
                right_lines.append((slope, intercept))
                right_weights.append(length)

    left_lane = np.dot(left_weights, left_lines) / np.sum(left_weights) if len(left_weights) > 0 else None
    right_lane = np.dot(right_weights, right_lines) / np.sum(right_weights) if len(right_weights) > 0 else None
    return left_lane, right_lane



def pixel_points(y1, y2, line):
    """Converts the slope and intercept of each line into pixel points."""
    if line is None:
        return None
    slope, intercept = line
    
    if slope == 0:  # Prevent division by zero
        return None
    
    # Calculate x1 and x2 for the given y1 and y2
    x1 = int((y1 - intercept) / slope)
    x2 = int((y2 - intercept) / slope)
    
    return ((x1, int(y1)), (x2, int(y2)))


def lane_lines(image, lines):
    """Create full length lines from pixel points."""
    left_lane, right_lane = average_slope_intercept(lines)
    y1 = image.shape[0]
    y2 = y1 * 0.60
    
    left_line = pixel_points(y1, y2, left_lane)
    right_line = pixel_points(y1, y2, right_lane)
    
    return left_line, right_line
 

def draw_lane_lines(image, lines, color=[0, 0, 255], thickness=15):
    """Draw lines, midpoints, and their coordinates onto the input image."""
    line_image = np.zeros_like(image)  # Create a blank image for drawing lines

    for line in lines:
        if line is not None:
            # Unpack the start and end points of the line
            start_point, end_point = line
            
            # Draw the line
            cv2.line(line_image, start_point, end_point, color, thickness)
            
            # Calculate the midpoint
            midpoint = ((start_point[0] + end_point[0]) // 2, (start_point[1] + end_point[1]) // 2)
            
            # Draw the midpoint
            cv2.circle(line_image, midpoint, 5, (0, 255, 0), -1)  # Green dot for midpoint
            
            # Add text annotations for the coordinates
            cv2.putText(line_image, f"Start: {start_point}", (start_point[0] + 10, start_point[1] - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
            cv2.putText(line_image, f"End: {end_point}", (end_point[0] + 10, end_point[1] - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
            cv2.putText(line_image, f"Mid: {midpoint}", (midpoint[0] + 10, midpoint[1] - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)

    # Overlay the line image onto the original image
    return cv2.addWeighted(image, 1.0, line_image, 1.0, 0.0)


# def frame_processor(frame):
#     """Process a single frame to detect lane lines."""
#     color_select = HSL_color_selection(frame)
#     gray = gray_scale(color_select)
#     smooth = gaussian_smoothing(gray)
#     edges = canny_detector(smooth)
#     region = region_selection(edges)
#     cv2.imshow('Region of interest', region)
#     lines = hough_transform(region)
#     left_line, right_line = lane_lines(frame, lines)
#     return draw_lane_lines(frame, [left_line, right_line])

# Global variables to track midpoints

left_region = True
right_region = False
Obstacle = False
LastCommandRan = 3

def frame_processor(image):
    """
    Process the input frame to detect lane lines.
        Parameters:
            image: Single video frame.
    """
    color_select = HSL_color_selection(image)
    gray = gray_scale(color_select)
    smooth = gaussian_smoothing(gray)
    edges = canny_detector(smooth)
    region = region_selection(edges)

    cv2.imshow('Region of interest with 1st triangle', region)
    hough = hough_transform(region)
    left_line, right_line = lane_lines(image, hough)

    global Obstacle, left_region, right_region, LastCommandRan





    if left_line is not None and right_line is None:
        left_region = True
        start_point_left, end_point_left = left_line
        midpoint_left = (
        (start_point_left[0] + end_point_left[0]) // 2,  # x-coordinate
        (start_point_left[1] + end_point_left[1]) // 2   # y-coordinate
         )
        print(midpoint_left,"Only left line exists.")
        #print("Previous point: ",prev_midpoint_left,"Current point: ",midpoint_left)

    if left_line is None and right_line is not None:   
        right_region = True
        start_point_right, end_point_right = right_line
        midpoint_right = (
        (start_point_right[0] + end_point_right[0]) // 2,  # x-coordinate
        (start_point_right[1] + end_point_right[1]) // 2   # y-coordinate
        )
        print(midpoint_right,"Only Right line exists.")
    if left_line is not None and right_line is not None:   
            # Calculate midpoints for left and right lines
        start_point_left, end_point_left = left_line
        start_point_right, end_point_right = right_line
        midpoint_left = (
        (start_point_left[0] + end_point_left[0]) // 2,  # x-coordinate
        (start_point_left[1] + end_point_left[1]) // 2   # y-coordinate
         )

        midpoint_right = (
        (start_point_right[0] + end_point_right[0]) // 2,  # x-coordinate
        (start_point_right[1] + end_point_right[1]) // 2   # y-coordinate
        )
        print("Both lanes exist.. left midpoint:",midpoint_left," right midpoint:",midpoint_right)

    # Draw lane lines for visualization
    result = draw_lane_lines(image, [left_line, right_line])

    


    if Obstacle is False:
        #this is the case for both when both lanes are present - BOTH LANES
        if left_line is not None and right_line is not None:
            print("Both lanes command ")     
            send_command("FORWARD")
            time.sleep(1)

        # #left line and no right line - WHEN LEFT LANE IS PRESENT
        if left_line is not None and right_line is None:
            if midpoint_left[0] >= 179:
                print("Too close to left .. Turn right")
                send_command("RIGHT")
                time.sleep(0.80)

            elif (midpoint_left[0]) < 179:
                print("Continue along left line")
                send_command("FORWARD")
                time.sleep(1) 


        # #right line and no left line
        elif right_line is not None and left_line is None:
            if midpoint_right[0] >= 430:
                print("Continue along right line")
                send_command("FORWARD")
                time.sleep(1)          
                #send_command("STOP")
                # forwardFlag = False

            elif midpoint_right[0] < 430:
                print("Too close to Right line... move left")
                send_command("LEFT")
                #send_command("RIGHT")
                #webbrowser.open(left_url,new=0)
                time.sleep(0.8)
            
        elif left_line is None and right_line is None:
            print("No lanes exist. Vehicle stopped!")
            send_command("FORWARD")
            time.sleep(1)  
            #send_command("STOP")
    
    elif Obstacle is True and (left_line is not None or right_line is not None):
        if left_region is True:
            left_region = False
            send_command("RIGHT")
            LastCommandRan = 1
            time.sleep(0.8)
        if right_region is True:
            right_region = False
            send_command("LEFT") 
            LastCommandRan = 2
            time.sleep(0.8)   
        
    elif Obstacle is True and left_line is None and right_line is None:
        if LastCommandRan == 1:
            send_command("RIGHT")
            time.sleep(0.8) 
        if LastCommandRan == 2:
            send_command("LEFT") 
            time.sleep(0.8) 

    # Save current midpoints for the next iteration


    return result

def webcam_video_processing():

    """Capture video from the webcam and process it for lane detection."""
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break

        processed_frame = frame_processor(frame)
        cv2.imshow('Lane Detection', processed_frame)
        #print("Tab Count is: ", TabCount)
        #if TabCount >= 20:
            #time.sleep(10)
            #kill_firefox()
            #TabCount = 0

        if cv2.waitKey(5) & 0xFF == ord('q'):
            #webbrowser.open(stop_url,new=0)
            send_command("STOP")
            break

    cap.release()
    cv2.destroyAllWindows()


def start_ros_node():
    """Start the ROS node for obstacle detection."""
    rclpy.init()
    minimal_subscriber = MinimalSubscriber()
    try:
        rclpy.spin(minimal_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        minimal_subscriber.destroy_node()
        rclpy.shutdown()

#should there be a listener in this code?

def main():
    """Run the main program."""
    ros_thread = threading.Thread(target=start_ros_node, daemon=True)
    ros_thread.start()

    webcam_video_processing()


if __name__ == '__main__':
    main()
