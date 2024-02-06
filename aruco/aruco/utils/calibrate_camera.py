import cv2
import numpy as np

# Define the size of the chessboard (number of inner corners)
chessboard_size = (9, 6)

# Criteria for termination of the iterative process of corner refinement
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# Prepare object points, like (0,0,0), (1,0,0), (2,0,0) ..., (8,5,0)
objp = np.zeros((chessboard_size[0] * chessboard_size[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:chessboard_size[0], 0:chessboard_size[1]].T.reshape(-1, 2)

# Arrays to store object points and image points from all images
objpoints = []  # 3D points in real world space
imgpoints = []  # 2D points in image plane

# Capture video from the camera
cap = cv2.VideoCapture(4)  # Change the argument to the camera index if using an external camera
while True:
    import time
    time.sleep(1)
    ret, frame = cap.read()

    if not ret:
        print("Failed to capture frame")
        break



    # Convert the frame to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    cv2.imshow('Chessboard Calibration', gray)
    # Find the chessboard corners
    # ret, corners = cv2.findChessboardCornersS(gray, chessboard_size, None, flags=cv2.CALIB_USE_INTRINSIC_GUESS)
    ret, corners = cv2.findChessboardCorners(gray, (9, 6), flags=cv2.CALIB_CB_ADAPTIVE_THRESH +
                                               cv2.CALIB_CB_FAST_CHECK +
                                               cv2.CALIB_CB_NORMALIZE_IMAGE)
    print(ret)
    # If found, add object points, image points (after refining them)
    if ret:
        objpoints.append(objp)

        corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        imgpoints.append(corners2)

        # Draw and display the corners
        cv2.drawChessboardCorners(frame, chessboard_size, corners2, ret)
        cv2.imshow('Chessboard Calibration', frame)

    # Exit on 'ESC' key press
    if cv2.waitKey(1) & 0xFF == 27:
        break

# Release the capture
cap.release()
cv2.destroyAllWindows()

# Perform camera calibration
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

# Save calibration results to a file
np.savez('camera_calibration.npz', mtx=mtx, dist=dist, rvecs=rvecs, tvecs=tvecs)

print("Calibration complete. Calibration results saved to 'camera_calibration.npz'")
