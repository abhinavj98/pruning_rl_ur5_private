import cv2
import cv2.aruco as aruco
import matplotlib.pyplot as plt
import matplotlib.patches as patches

# Function to generate and save ArUco board
def generate_aruco_board(board_size, marker_size, margin_size, output_path):
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
    board = aruco.CharucoBoard.generateImage(board_size, (marker_size, marker_size), aruco_dict, (margin_size, margin_size))
    board_image = board.draw((2000, 2000))
    cv2.imwrite(output_path, board_image)

    # Display the ArUco board image using matplotlib (optional)
    plt.imshow(cv2.cvtColor(board_image, cv2.COLOR_BGR2RGB))
    plt.show()

# Set board parameters
board_size = (5, 7)  # Number of squares in rows and columns
marker_size = 0.04  # Size of each marker in meters
margin_size = 0.02  # Size of the margin around the board in meters
output_path = 'aruco_board.png'  # Output path for the ArUco board image

# Generate and save the ArUco board
generate_aruco_board(board_size, marker_size, margin_size, output_path)
