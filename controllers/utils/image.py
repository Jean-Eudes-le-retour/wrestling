import numpy as np
import cv2
import base64

def locate_opponent(img):
    """Locate the opponent robot in the image."""
    # the robot is supposed to be located at a concentration of multiple color changes (big Laplacian values)
    laplacian = cv2.Laplacian(img, cv2.CV_8U, ksize=3)
    # those spikes are then smoothed out using a Gaussian blur to get blurry blobs
    blur = cv2.GaussianBlur(laplacian, (0, 0), 2)
    # we apply a threshold to get a binary image of potential robot locations
    gray = cv2.cvtColor(blur, cv2.COLOR_BGR2GRAY)
    _, thresh = cv2.threshold(gray, 80, 255, cv2.THRESH_BINARY)
    # the binary image is then dilated to merge small groups of blobs together
    closing = cv2.morphologyEx(
        thresh, cv2.MORPH_CLOSE, cv2.getStructuringElement(cv2.MORPH_RECT, (15, 15)))
    # the robot is assumed to be the largest contour
    largest_contour = get_largest_contour(closing)
    # we get its centroid for an approximate opponent location
    cx, cy = get_contour_centroid(largest_contour, img)
    return largest_contour, cx, cy

def send_image_to_robot_window(robot, img):
    """Send an openCV image to the robot's web interface."""
    _, im_arr = cv2.imencode('.png', img[:,:,:3])
    im_bytes = im_arr.tobytes()
    im_b64 = base64.b64encode(im_bytes).decode()
    robot.wwiSendText("data:image/png;base64," + im_b64)

def get_cv_image_from_camera(camera):
    """Get an openCV image from a Webots camera."""
    return np.frombuffer(camera.getImage(), np.uint8).reshape((camera.getHeight(), camera.getWidth(), 4))

def get_largest_contour(image):
    """Get the largest contour in an image."""
    contours, _ = cv2.findContours(image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    contours = sorted(contours, key=cv2.contourArea, reverse=True)
    if len(contours) == 0:
        return None
    return contours[0]

def get_contour_centroid(contour, img):
    """Get the centroid of a contour.
    
    If the contour is None, return the center of the image."""
    try:
        M = cv2.moments(contour)
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])
    except:
        cx = img.shape[1]//2
        cy = img.shape[0]//2
    return cx, cy