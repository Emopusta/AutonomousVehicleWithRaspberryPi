
import cv2

# Load image
img = cv2.imread('deneme.jpg', cv2.IMREAD_GRAYSCALE)

# Apply Gaussian blur to the image to reduce noise
img_blur = cv2.GaussianBlur(img, (5, 5), 0)

# Apply Canny edge detection algorithm
edges = cv2.Canny(img, 400, 400)

# Display the original image and the detected edges
cv2.imshow('Original Image', img)
cv2.imshow('Edges', edges)

# Wait for a key press and then exit
cv2.waitKey(0)
cv2.destroyAllWindows()