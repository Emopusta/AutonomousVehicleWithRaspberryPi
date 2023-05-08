"""
import numpy as np
import cv2 as cv
from matplotlib import pyplot as plt
img = cv.imread('edgedet.jpg',0)
edges = cv.Canny(img,500,200)
plt.subplot(121),plt.imshow(img,cmap = 'gray')
plt.title('Original Image'), plt.xticks([]), plt.yticks([])
plt.subplot(122),plt.imshow(edges,cmap = 'gray')
plt.title('Edge Image'), plt.xticks([]), plt.yticks([])
plt.show()
"""
import sys
import math
import cv2 as cv
import numpy as np

def deneme(inp_rho,inp_theta,cdst):
    rho = inp_rho
    theta = inp_theta
    a = math.cos(theta)
    b = math.sin(theta)
    x0 = a * rho
    y0 = b * rho
    pt1 = (int(x0 + 1000*(-b)), int(y0 + 1000*(a)))
    pt2 = (int(x0 - 1000*(-b)), int(y0 - 1000*(a)))
    cv.line(cdst, pt1, pt2, (255,0,0), 3, cv.LINE_AA,)
    #cv.imshow("Detected Lines (in red) - Standard Hough Line Transform", cdst)


def main(argv):
    
    default_file = '/Users/emred/Desktop/otonom/deneme.jpg'
    filename = argv[0] if len(argv) > 0 else default_file
    # Loads an image
    src = cv.imread(cv.samples.findFile(filename), cv.IMREAD_GRAYSCALE)
    src = src[500:1000,500:1000]
    # Check if image is loaded fine
    if src is None:
        print ('Error opening image!')
        print ('Usage: hough_lines.py [image_name -- default ' + default_file + '] \n')
        return -1
    
    
    dst = cv.Canny(src,150, 150, None, 3)
    
    # Copy edges to the images that will display the results in BGR
    cdst = cv.cvtColor(dst, cv.COLOR_GRAY2BGR)
    cdstP = np.copy(cdst)
    
    lines = cv.HoughLines(dst, 1, np.pi / 180, 50, None, 0, 0)
    
    

    if lines is not None:
        for i in range(0, len(lines)):
            rho = lines[i][0][0]
            theta = lines[i][0][1]
            a = math.cos(theta)
            b = math.sin(theta)
            x0 = a * rho
            y0 = b * rho
            pt1 = (int(x0 + 1000*(-b)), int(y0 + 1000*(a)))
            pt2 = (int(x0 - 1000*(-b)), int(y0 - 1000*(a)))
            #cv.line(cdst, pt1, pt2, (0,0,255), 3, cv.LINE_AA)
            
            
    
    
    linesP = cv.HoughLinesP(dst, 1, np.pi / 180, 50, None, 50, 10)
    
    if linesP is not None:
        for i in range(0, len(linesP)):
            l = linesP[i][0]
            cv.line(cdstP, (l[0], l[1]), (l[2], l[3]), (0,0,255), 3, cv.LINE_AA)

    #cv.imshow("dst",dst);
   # cv.imshow("Source", src)
    # liness = [lines[0], lines[2]]
    # print(liness)
    # for line in liness:
    #     rho, theta = line[0]
    #     a = np.cos(theta)
    #     b = np.sin(theta)
    #     x0 = a * rho
    #     y0 = b * rho
    #     x1 = int(x0 + 1000 * (-b))
    #     y1 = int(y0 + 1000 * (a))
    #     x2 = int(x0 - 1000 * (-b))
    #     y2 = int(y0 - 1000 * (a))
    #     cv.line(cdst, (x1, y1), (x2, y2), (255, 0, 0), 2)

    

    #deneme(lines, cdst)

    right_lines = []
    left_lines = []
    for i in lines:
        slope = i[0][0]
        intercept = i[0][1]
        if slope<0:
            right_lines.append((slope,intercept))
        else:
            left_lines.append((slope,intercept))

    averaged_right = np.average(right_lines, axis=0)
    averaged_left =np.average(left_lines, axis=0)
    
    
    

    
    theta1 = averaged_right[1]
    theta2 = averaged_left[1]
    rho1 = averaged_right[0]
    rho2 = averaged_left[0]
    a = np.array([[np.cos(theta1), np.sin(theta1)], [np.cos(theta2), np.sin(theta2)]])
    b = np.array([[rho1], [rho2]])
    intersection = np.linalg.solve(a, b)
    x, y = map(int, intersection)
    print(x, y)
    print(cdst.shape)
    cv.line(cdst, (x, y), (200, 440), (0, 255, 0), 2)
  
    middle_point = (250,250)
    cv.circle(cdst,middle_point,10,(0,255,0),-1)

    p1 = np.array([x,y])
    p2 = np.array([200,440])
    p3 = np.array([250,250])

    #orta nokta bulma
    d=np.cross(p2-p1,p3-p1)/np.linalg.norm(p2-p1)

    print(d)

    deneme(averaged_right[0],averaged_right[1],cdst)

    deneme(averaged_left[0],averaged_left[1],cdst)

    cv.imshow("Detected Lines (in red) - Standard Hough Line Transform", cdst)
    #cv.imshow("Detected Lines (in red) - Probabilistic Line Transform", cdstP)
    
    cv.waitKey()
    return 0
    
if __name__ == "__main__":
    main(sys.argv[1:])
