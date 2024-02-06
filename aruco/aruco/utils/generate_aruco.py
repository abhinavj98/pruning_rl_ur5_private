import cv2
import cv2.aruco as aruco
import numpy as np

# Define the dictionary and marker size
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
marker_size = 500  # Adjust the size as needed

# Generate ArUco markers with different IDs
marker_id_1 = 0
marker_id_2 = 1

# Create an empty canvas for the markers
canvas_size = marker_size*2
canvas = np.ones((canvas_size, canvas_size), dtype=np.uint8) * 255

# Draw ArUco markers on the canvas
marker_1 = aruco_dict.generateImageMarker(marker_id_1, marker_size)
marker_2 = aruco_dict.generateImageMarker(marker_id_2, marker_size)

# Place markers on the canvas at different positions
canvas[50:50+marker_size, 50:50+marker_size] = marker_1
canvas[200:200+marker_size, 200:200+marker_size] = marker_2

# Save the generated markers as images
cv2.imwrite('aruco_marker_0.png', marker_1)
cv2.imwrite('aruco_marker_1.png', marker_2)

# Display the canvas with markers (optional)
cv2.imshow('ArUco Markers', canvas)
cv2.waitKey(0)
cv2.destroyAllWindows()
