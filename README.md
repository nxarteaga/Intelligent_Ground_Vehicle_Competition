# Lane Detection System

This project implements a lane detection system that processes video frames to detect lane lines. The program uses Python, OpenCV, and other libraries to perform image processing, edge detection, and lane detection. It is designed to process webcam video input and identify lane lines in real time.

## Features

Edge Detection: Utilizes the Canny edge detection algorithm to identify edges in video frames.

Region of Interest (ROI): Applies a triangular mask to focus on the lane area.

HSL Color Filtering: Filters the image to isolate white lane lines using HSL (Hue, Saturation, Lightness) color space.

Lane Line Approximation: Detects and averages lane line slopes and positions using Hough Transform.

Real-Time Processing: Processes video frames from a webcam and displays the detected lanes with FPS (frames per second) information.

Command Sending: Includes functionality to send movement commands to an ESP32 device (commented by default).

## Prerequisites

Python 3.7+

numpy

opencv-python

matplotlib

Pillow

moviepy

requests

You can install the required dependencies using:

pip install numpy opencv-python matplotlib Pillow moviepy requests

## How It Works

Color Filtering: Converts the RGB image to HSL and filters white lane lines.

Grayscale Conversion: Converts the filtered image to grayscale for simplicity.

Gaussian Smoothing: Reduces noise using Gaussian blur.

Edge Detection: Identifies edges using the Canny algorithm.

Region of Interest: Masks areas outside the lane region.

Hough Transform: Detects lane lines based on edges.

Lane Drawing: Draws lane lines on the original frame.

Real-Time Processing: Displays the processed video with FPS information.

## Running the Code

Connect Webcam: Ensure a webcam is connected to your computer.

Run the Program:

python lane_detection.py

Real-Time Lane Detection: The video stream will open, showing lane detection results.

Exit: Press the q key to stop the program.

## Code Structure

Image Processing Functions:

convert_hsl(image): Converts an image from RGB to HSL.

HSL_color_selection(image): Filters the image to isolate white lane lines.

gray_scale(image): Converts the image to grayscale.

gaussian_smoothing(image): Applies Gaussian blur to reduce noise.

canny_detector(image): Applies Canny edge detection.

## Lane Detection Functions:

region_selection(image): Masks areas outside the region of interest.

hough_transform(image): Detects lines using Hough Transform.

average_slope_intercept(lines): Averages the detected lines.

pixel_points(y1, y2, line): Converts slopes and intercepts to pixel points.

lane_lines(image, lines): Creates lane lines.

draw_lane_lines(image, lines): Draws lines on the image.

### Webcam Processing:

webcam_video_processing(): Captures webcam video and processes each frame.

### ESP32 Communication:

send_command_to_esp32(url): Sends HTTP commands to the ESP32 (commented by default).

## Customization

Adjust Canny Thresholds:
Modify low_threshold and high_threshold in the canny_detector function for edge sensitivity.

Change ROI Shape:
Update the top_left, top_right, bottom_left, and bottom_right coordinates in region_selection to adjust the region of interest.

Hough Transform Parameters:
Modify rho, theta, threshold, minLineLength, and maxLineGap in the hough_transform function to tweak line detection.

## Known Issues

Performance may vary depending on the quality of the webcam and lighting conditions.

Lane detection may fail if the lane lines are faded or obscured.

## Future Improvements

Add support for yellow lane detection.

Implement lane curvature detection.

Integrate machine learning for robust lane tracking.

Optimize for low-light conditions.

## License

This project is open-source and available under the MIT License.

Acknowledgments

