# function.py for line-tracking jinpao
import os
os.environ["OPENCV_VIDEOIO_MSMF_ENABLE_HW_TRANSFORMS"] = "0"
import cv2
import numpy as np

def LinetrackCam_Open():
    cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 2)
    # shutter_speed = 1/50  # 100/hz
    # cap.set(cv2.CAP_PROP_EXPOSURE, shutter_speed)#win ,<= -6 #inliux 1/s
    return cap

def BGR2BIN(frame,threshold = 95):
    # Create a copy of the frame and convert it to grayscale
    frame = cv2.GaussianBlur(frame,(23,23),cv2.BORDER_DEFAULT)
    imgray_array = cv2.cvtColor(frame.copy(), cv2.COLOR_BGR2GRAY)
    
    max_value = 255        
    _, binary_image = cv2.threshold(imgray_array, threshold, max_value, cv2.THRESH_BINARY)
    return binary_image

def horizontal(x_pos,binrary_image):
    apos = 999
    # Extract a single row from the binary image
    binary_image = binrary_image[x_pos:x_pos+1, :]
    binary_image_index = np.where(np.array(binary_image[0]) == 0)
    height, width = binrary_image.shape
    # Extract 8 sections from the binary image using NumPy
    section_width = width // 8
    binary_image_8bit = []

    for i in range(8):
        section = binary_image[:, i * section_width:(i + 1) * section_width]
        binary_value = 1 if np.any(section == 0) else  0
        binary_image_8bit.append(binary_value)

    # Calculate the average position of white pixels in the row
    if len(binary_image_index[0]) > 0:
        apos = np.average(binary_image_index[0])

        # Draw a circle at the average position
        # cv2.circle(binary_image, (np.round(apos).astype(np.int32), 0), 5, (255, 255, 255), -1)

    # Resize the binary image for display
    # binary_images = cv2.resize(binary_image, (width, 10), interpolation=cv2.INTER_AREA)
    # cv2.imshow("Binary Image "+str(x_pos), binary_images)

    return np.array2string(np.round(apos).astype(np.int32),separator='') +':'+ ''.join(map(str, binary_image_8bit))

def vertical(y_pos,binary_image):
    apos = 999
    # Get the height and width of the image
    height, width = binary_image.shape
    # Extract a single column from the binary image
    binary_image = binary_image[:, y_pos:y_pos+1]
    binary_image_index = np.where(np.array(binary_image[:, 0]) == 0)
    
    # Extract 8 sections from the binary image using NumPy
    section_height = height // 8
    binary_image_8bit = []

    for i in range(8):
        section = binary_image[i * section_height:(i + 1) * section_height, :]
        binary_value = 1 if np.any(section == 0) else 0
        binary_image_8bit.append(binary_value)

    # Calculate the average position of white pixels in the column
    if len(binary_image_index[0]) > 0:
        apos = np.average(binary_image_index[0])

        # Draw a circle at the average position
        # cv2.circle(binary_image, (y_pos, np.round(apos).astype(np.int32)), 5, (255, 255, 255), -1)


    # # Resize the binary image for display
    # binary_images = cv2.resize(binary_image, (10, height), interpolation=cv2.INTER_AREA)
    # cv2.imshow("Binary Image " + str(y_pos), binary_images)

    return np.array2string(np.round(apos).astype(np.int32), separator='') + ':' + ''.join(map(str, binary_image_8bit))


# def auto_adjust_brightness_color(image):
#     # Split the color image into its RGB channels
#     b, g, r = cv2.split(image)

#     # Apply histogram equalization to each channel
#     r_eq = cv2.equalizeHist(r)
#     g_eq = cv2.equalizeHist(g)
#     b_eq = cv2.equalizeHist(b)

#     # Merge the equalized channels back into a color image
#     adjusted_image = cv2.merge((b_eq, g_eq, r_eq))

#     return adjusted_image


