##### the script for saving the image dedicated for the calibration #########

import cv2
import numpy as np
from pathlib import Path
from rosbags.highlevel import AnyReader
from rosbags.serde import deserialize_cdr
from sensor_msgs.msg import Image

# Path to your ROS 2 bag
bag_path = Path('./TestBag')

# Set the frame delay for video playback (in milliseconds)
frame_delay = 30  # 30 ms delay for ~33 frames per second

# Function to display images as a video stream
def display_video(image_data, encoding, width, height):
    """Decode the image data and show it using OpenCV (as video)."""
    if encoding == "rgb8":
        img_array = np.frombuffer(image_data, dtype=np.uint8).reshape((height, width, 3))
    elif encoding == "mono8":
        img_array = np.frombuffer(image_data, dtype=np.uint8).reshape((height, width))
    elif encoding == "bgr8":  # Add support for bgr8 encoding
        img_array = np.frombuffer(image_data, dtype=np.uint8).reshape((height, width, 3))
    else:
        raise ValueError(f"Unsupported encoding: {encoding}")

    # Display the image
    cv2.imshow("Camera Video", img_array)
    
    return img_array

# --- Read bag and display video ---
with AnyReader([bag_path]) as reader:
    
    
    for connection, timestamp, rawdata in reader.messages():
        if connection.topic == '/oakd/rgb/preview/image_raw':  # Change topic name if needed
            # Deserialize the message
            msg = deserialize_cdr(rawdata, connection.msgtype)
            
            # Print out basic details about the image
            print(f"Timestamp: {timestamp}")
            print(f"Encoding: {msg.encoding}")
            print(f"Width: {msg.width}, Height: {msg.height}")
            
            # Display the image (video playback)
            img_array = display_video(msg.data, msg.encoding, msg.width, msg.height)

            # Wait for a keypress and control the video speed using waitKey
            key = cv2.waitKey(frame_delay) & 0xFF  # Adjust frame delay to control speed
            if key == ord('s'):  # If 's' is pressed, save the current image
                timestamp_str = str(timestamp)
                filename = f"saved_image_{timestamp_str}.png"
                cv2.imwrite(filename, img_array)
                print(f"Image saved as {filename}")

            elif key == ord('q'):  # If 'q' is pressed, quit
                break  # Stop the video if 'q' is pressed

    # Close the OpenCV window after finishing
    cv2.destroyAllWindows()