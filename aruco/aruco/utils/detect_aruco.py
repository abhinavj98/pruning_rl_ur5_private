import cv2
import cv2.aruco as aruco
import numpy as np

# Define the ArUco dictionary and parameters
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
parameters =  cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)

#load camera matrix and distortion coefficients from file
cameraMatrixParam = np.load('camera_calibration.npz')
cameraMatrix = cameraMatrixParam['mtx']
distCoeffs = cameraMatrixParam['dist']

print(cameraMatrix)
# Capture video from the camera
cap = cv2.VideoCapture(4)  # Change the argument to the camera index if using an external camera

while True:
    ret, frame = cap.read()

    if not ret:
        print("Failed to capture frame")
        break

    # Convert the frame to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Detect ArUco markers
    corners, ids, _ = detector.detectMarkers(gray)

    # Draw detected markers on the frame
    if ids is not None:
        aruco.drawDetectedMarkers(frame, corners, ids)

        # Estimate pose of detected markers
        rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, 300, cameraMatrix, distCoeffs)

        # Draw axes for each detected marker
        for i in range(len(ids)):
            # result_img = cv2.drawFrameAxes(dst1, mtx, dist, rvec[i, :, :], tvec[i, :, :], 0.03)
            cv2.drawFrameAxes(frame, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 300)

    # Display the frame
    cv2.imshow('ArUco Marker Detection', frame)

    # Exit on 'ESC' key press
    if cv2.waitKey(1) & 0xFF == 27:
        break

# Release the capture
cap.release()
cv2.destroyAllWindows()
