import matplotlib.pyplot as plt
from PIL import Image
import numpy as np
import time
import os
import cv2  # Import OpenCV
from moviepy.editor import VideoFileClip  # Import VideoFileClip\
import requests
import serial_communication
import serial
import threading
#from TestESP32 import Move_Bot


#Define function motorControl
def sendCommand(command):

    ser = serial.Serial('/dev/serial0',baudrate=115200,timeout=1)
    ser.setRTS(False)
    ser.setDTR(False)

    try:
        #send command
        ser.write(command.encode()+ b'\n')
        # Read response from the serial port
        data = ser.readline().decode('utf-8')
        if data:
            print(f"Received: {data}", end='')

        # Clear the input buffer after reading the response
        ser.reset_input_buffer()
    
    finally:
        # Ensure the serial port is closed after communication
        ser.close()





# Define the function to display images
def list_images(images, cols=2, rows=None, cmap=None, output_file='output_images.png'):
    num_images = len(images)
    if rows is None:
        rows = (num_images + cols - 1) // cols  # Ceiling division

    plt.figure(figsize=(10, 11))
    
    for i, image in enumerate(images):
        plt.subplot(rows, cols, i + 1)
        cmap_to_use = 'gray' if len(image.shape) == 2 else cmap
        plt.imshow(image, cmap=cmap_to_use)
        plt.xticks([])
        plt.yticks([])

    plt.tight_layout(pad=0, h_pad=0, w_pad=0)
    
    # Save the figure to a file
    plt.savefig(output_file)  # Specify the output filename
    plt.close()  # Close the figure to free memory

def convert_hsl(image):
    """Convert an image from RGB to HSL."""
    return cv2.cvtColor(image, cv2.COLOR_RGB2HLS)

def HSL_color_selection(image):
    """Apply color selection to the HSL images to blackout everything except for white lane lines."""
    converted_image = convert_hsl(image)
    
    # White color mask
    lower_threshold = np.uint8([0, 200, 0])  # H, L, S
    upper_threshold = np.uint8([245, 245, 245])
    white_mask = cv2.inRange(converted_image, lower_threshold, upper_threshold)
    
    # Combine masks (only white in this case)
    masked_image = cv2.bitwise_and(image, image, mask=white_mask)
    
    return masked_image

def gray_scale(image):
    """Convert an image to grayscale."""
    return cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)

def gaussian_smoothing(image, kernel_size=5):
    """Apply Gaussian smoothing to the image."""
    return cv2.GaussianBlur(image, (kernel_size, kernel_size), 0)

def canny_detector(image, low_threshold=50, high_threshold=150):
    """Apply Canny edge detection to the image."""
    return cv2.Canny(image, low_threshold, high_threshold)


#DEBUG trying to redefined a bigger triangle 
def canny(image):
    if image is None:
        cap.release()
        cv2.destroyAllWindows()
        exit()
    gray = cv2.cvtColor(image,cv2.COLOR_BAYER_BG2BGR)
    kernel = 5
    blur =cv2.GaussianBlur(gray,(kernel,kernel),0)
    canny = cv2.Canny(gray,50,150)
    return canny


def region_selection(image):
    """Determine and cut the region of interest in the input image."""
    mask = np.zeros_like(image)   
    if len(image.shape) > 2:
        channel_count = image.shape[2]
        ignore_mask_color = (255,) * channel_count
    else:
        ignore_mask_color = 255
    
    rows, cols = image.shape[:2]
    
    ### Changes made only here
    top_left     = [cols * 0.3, rows * 0.60]
    top_right    = [cols * 0.7, rows * 0.60]
    bottom_left  = [cols * 0.01, rows * 0.90]  
    bottom_right = [cols * 0.99, rows * 0.90]
    
    #This order of this array is what defines the shape of the window. Just had to change the order inside the array
    vertices = np.array([[bottom_left, top_left, bottom_right, top_right]], dtype=np.int32)

    cv2.fillPoly(mask, vertices, ignore_mask_color)
    masked_image = cv2.bitwise_and(image, mask)
    
    return masked_image

def hough_transform(image):
    """Determine lines in the image using the Hough Transform."""
    rho = 1              # Distance resolution of the accumulator in pixels.
    theta = np.pi / 180  # Angle resolution of the accumulator in radians.
    threshold = 50       # Only lines that are greater than threshold will be returned. Higher the number, fewer lines detected
    minLineLength = 10   # Line segments shorter than that are rejected. Minimum length of line to be detected
    maxLineGap = 100     # Maximum allowed gap between points on the same line to link them. Max gap allowed betweeen the same line to be detected
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
 
# testing for center
# def get_lane_center(left_line, right_line, image_width):
#     if left_line is None or right_line is None:
#         return image_width / 2  # Default to center if lines aren't detected
    
#     # Get the midpoint of the lane
#     left_x = (left_line[0][0] + left_line[1][0]) / 2  # Average x-coordinates of left lane
#     right_x = (right_line[0][0] + right_line[1][0]) / 2  # Average x-coordinates of right lane
#     #print("Left is", left_x, "and Right is", right_x)
    
#     lane_center = (left_x + right_x) / 2
    
#     return lane_center













# def draw_lane_lines(image, lines, color=[0, 0, 255], thickness=15):
#     """Draw lines and their coordinates onto the input image."""
#     line_image = np.zeros_like(image)  # Create a blank image for drawing lines

#     for line in lines:
#         if line is not None:
#             # Unpack the start and end points of the line
#             start_point, end_point = line
            
#             # Draw the line
#             cv2.line(line_image, start_point, end_point, color, thickness)
            
#             # Add text annotations for the coordinates
#             cv2.putText(line_image, f"Start: {start_point}", start_point,
#                         cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
#             cv2.putText(line_image, f"End: {end_point}", end_point,
#                         cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)

#     # Overlay the line image onto the original image
#     return cv2.addWeighted(image, 1.0, line_image, 1.0, 0.0)


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


#Some logic refining needed
# def draw_lane_lines(image, lines, color=[0, 0, 255], thickness=12):
#     """Draw lines and the midpoint onto the input image."""
#     line_image = np.zeros_like(image)
#     mid_point = None  # Initialize the midpoint variable
#     points = []

#     for line in lines:
#         if line is not None:
#             start_point, end_point = line
#             cv2.line(line_image, start_point, end_point, color, thickness)
            
#             # Add both start and end points to `points` for averaging
#             points.extend([start_point, end_point])

#             # Calculate midpoint between left and right end points
#             # if mid_point is None:
#             #     mid_point = np.array(end_point)  # Initialize mid_point with the first end_point
#             # else:
#             #     mid_point = (mid_point + np.array(end_point)) // 2  # Average of the points for midpoint

#             # Annotate the start and end points on the image
#             cv2.putText(line_image, f"Start: {start_point}", start_point,
#                         cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1, cv2.LINE_AA)
#             cv2.putText(line_image, f"End: {end_point}", end_point,
#                         cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1, cv2.LINE_AA)
    
#     # Draw midpoint if both lines were detected
#     # if mid_point is not None:
#     #     # Convert midpoint to standard integer format
#     #     mid_point = (int(mid_point[0]), int(mid_point[1]))

#     #     # Draw a circle at the midpoint
#     #     cv2.circle(line_image, mid_point, 5, (0, 255, 0), -1)
#     #     # Draw an "X" at the midpoint for additional visibility
#     #     cv2.drawMarker(line_image, mid_point, (0, 255, 0), markerType=cv2.MARKER_TILTED_CROSS, markerSize=15, thickness=2)
#     #     # Print the coordinates of the midpoint
#     #     print(f"Midpoint: {mid_point}")
#     #     # Optionally, display the coordinates on the image
#     #     cv2.putText(line_image, f"Midpoint: {mid_point}", (mid_point[0] + 10, mid_point[1] + 10),
#     #                 cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)
#     # Calculate and draw midpoint if there are points detected
#     if points:
#         # Convert points to numpy array for mean calculation
#         points = np.array(points)
#         mid_point = tuple(np.mean(points, axis=0).astype(int))

#         # Draw a circle and cross marker at the midpoint
#         cv2.circle(line_image, mid_point, 5, (0, 255, 0), -1)
#         cv2.drawMarker(line_image, mid_point, (0, 255, 0), markerType=cv2.MARKER_TILTED_CROSS, markerSize=15, thickness=2)

#         # Print the midpoint coordinates and annotate them on the image
#         print(f"Midpoint: {mid_point}")
#         cv2.putText(line_image, f"Midpoint: {mid_point}", (mid_point[0] + 10, mid_point[1] + 10),
#                     cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)
    
#     # Overlay the line image on the original image
#     return cv2.addWeighted(image, 1.0, line_image, 1.0, 0.0)





# Global variables to track midpoints
midpoint_left = None
midpoint_right = None
prev_midpoint_left = None
prev_midpoint_right = None
steeringThreshold = 10

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

    #cv2.imshow('Region of interest with 1st triangle', region)
    hough = hough_transform(region)
    left_line, right_line = lane_lines(image, hough)

    global prev_midpoint_left,prev_midpoint_right 
    # Reset midpoints
    midpoint_left = None
    midpoint_right = None


    if left_line is not None and right_line is None:
        
        start_point_left, end_point_left = left_line
        midpoint_left = (
        (start_point_left[0] + end_point_left[0]) // 2,  # x-coordinate
        (start_point_left[1] + end_point_left[1]) // 2   # y-coordinate
         )
        print("Previous point: ",prev_midpoint_left,"Current point: ",midpoint_left)

    if left_line is None and right_line is not None:   
        
        start_point_right, end_point_right = right_line
        midpoint_right = (
        (start_point_right[0] + end_point_right[0]) // 2,  # x-coordinate
        (start_point_right[1] + end_point_right[1]) // 2   # y-coordinate
        )
        print(midpoint_right,"Only Right line exist")
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
        print("Both lanes exist.. calculate midpoint")




    # Draw lane lines for visualization
    result = draw_lane_lines(image, [left_line, right_line])

    #make decision about moving 
    obstacle = False
    #this is the case for both when both lanes are present
    if left_line is not None and right_line is not None: 
        # sendCommand('{"T":1,"L":0.0,"R":0.0}')
        sendCommand('{"T":1,"L":0.25,"R":0.25}')
    # #left line and no right line
    if left_line is not None and right_line is None:
        if prev_midpoint_left is None or prev_midpoint_left == midpoint_left or (midpoint_left[0]) > (prev_midpoint_left[0]-steeringThreshold) and (midpoint_left[0]) < (prev_midpoint_left[0]+steeringThreshold) :
            print("Basic command for left line")
            # sendCommand('{"T":1,"L":0.0,"R":0.0}')
            sendCommand('{"T":1,"L":0.20,"R":0.20}')
            print("Continue along the same path")
        elif (midpoint_left[0]) > (prev_midpoint_left[0]+steeringThreshold):
            print("Move away left")
            # sendCommand('{"T":1,"L":0.0,"R":0.0}')
            sendCommand('{"T":1,"L":0.40,"R":-0.05}') #move away the left lane
            # sendCommand('{"T":1,"L":0.50,"R":0.0}') #move away the left lane
            # sendCommand('{"T":1,"L":0.50,"R":0.0}') #move away the left lane
            # sendCommand('{"T":1,"L":0.50,"R":0.0}') #move away the left lane
        elif (midpoint_left[0]) < (prev_midpoint_left[0]-steeringThreshold):
            print("Move toward left")
            # sendCommand('{"T":1,"L":0.0,"R":0.0}')
            sendCommand('{"T":1,"L":0.08,"R":0.20}') #move to the left lane
        
    # #right line and no left line
    # if right_line is not None and left_line is None: 
    #     sendCommand('{"T":1,"L":0.0,"R":0.0}')  
    #     sendCommand('{"T":1,"L":0.08,"R":0.30}')

    # #No lines, keep moving forward
    # if right_line is None and left_line is None:
    #     sendCommand('{"T":1,"L":0.0,"R":0.0}')
    #     sendCommand('{"T":1,"L":0.20,"R":0.20}')

    # #time.sleep(4.5)

    # Save current midpoints for the next iteration
    prev_midpoint_left = midpoint_left
    prev_midpoint_right = midpoint_right

    # sendCommand('{"T":1,"L":0.08,"R":0.08}')
    return result


# Function to calculate turn angle based on lane center deviation
def calculateTurnAngle(img, left_fit, right_fit):
    y_eval = img.shape[0]
    left_x = left_fit[0] * y_eval**2 + left_fit[1] * y_eval + left_fit[2]
    right_x = right_fit[0] * y_eval**2 + right_fit[1] * y_eval + right_fit[2]
    lane_center = (left_x + right_x) / 2
    image_center = img.shape[1] / 2
    angle_offset = np.arctan((lane_center - image_center) / y_eval) * (180 / np.pi)
    return round(angle_offset, 2)

# Function to display angle information on the result image
def displayTurnAngle(img, angle_offset):
    turn_direction = 'Left' if angle_offset < 0 else 'Right'
    text = f'Turn {turn_direction} by {abs(angle_offset)} degrees'
    img = cv2.putText(img, text, (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2, cv2.LINE_AA)
    return img


def webcam_video_processing():
    """Capture video from the webcam and process it for lane detection."""
    cap = cv2.VideoCapture(0)  # Use 0 for the default webcam
     # Set the video frame width and height
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break


        start_time = time.time()

        # Process the frame
        #time.sleep(2)
        processed_frame = frame_processor(frame)
        
        processing_time = time.time() - start_time
        fps = 1 / processing_time if processing_time > 0 else 0

                # Display FPS on the frame
        cv2.putText(processed_frame, f"FPS: {fps:.2f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 
                    1, (255, 0, 0), 2, cv2.LINE_AA)

        #serial_communication.start_serial_communication()
        #command = '{"T":1,"L":0.08,"R":0.08}'
        #serial_communication.send_command(command)
        # Display the resulting frame
        cv2.imshow('Lane Detection - White Lines Only', processed_frame)

        # Wait for the remaining time to complete 1 second
        #time.sleep(max(1.0 - processing_time, 0))


        # Break the loop on 'q' key press
        if cv2.waitKey(5) & 0xFF == ord('q'):
            sendCommand('{"T":1,"L":0.0,"R":0.0}')
            break

    cap.release()
    cv2.destroyAllWindows()

# Run the webcam processing
webcam_video_processing()
