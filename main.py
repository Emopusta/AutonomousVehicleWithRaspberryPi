from src.pinConfiguration.MotorPinConfiguration import  MotorPinConfiguration
from src.ImageProcessing import ImageProcessing
import sys
import cv2 as cv

def main(argv):

    imageProcessing = ImageProcessing("deneme.jpg")

    imageProcessing.BGRtoGrayScale()

    imageProcessing.ROI(500,1000,500,1000)

    imageProcessing.cannyEdgeDetection(150, 150)

    imageProcessing.grayScaleToBGRShowImage()

    imageProcessing.houghLineTransform(threshold=50)

    imageProcessing.findLineToTrack()

    imageProcessing.setStaticMiddlePoint(250,250)

    imageProcessing.findDistanceBetweenStaticMiddlePointAndTrackLine()

    imageProcessing.showImage()

    
    cv.waitKey()
    return 0
    

if __name__ == "__main__":
    main(sys.argv[1:])
