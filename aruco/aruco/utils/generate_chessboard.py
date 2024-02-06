import cv2
import numpy as np

# Define the size of the chessboard (number of inner corners)
chessboard_size = (7, 7)

# Size of each square in the chessboard (in pixels)
square_size = 200

# Create a black image
image = np.zeros((chessboard_size[1] * square_size, chessboard_size[0] * square_size), dtype=np.uint8)

# Fill the image with alternating white and black squares
for i in range(0, chessboard_size[1]):
    for j in range(0, chessboard_size[0]):
        if (i + j) % 2 == 0:
            image[i * square_size:(i + 1) * square_size, j * square_size:(j + 1) * square_size] = 255

# Save the generated chessboard pattern
cv2.imwrite('chessboard_pattern.png', image)

# Display the generated chessboard pattern (optional)
cv2.imshow('Chessboard Pattern', image)
cv2.waitKey(0)
cv2.destroyAllWindows()