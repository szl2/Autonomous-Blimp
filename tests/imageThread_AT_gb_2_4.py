# import sys
# sys.path.append("../")

from ESP32_AT.imageTread_AT import get_AT_6DOF_info

from constants import *
import ball_detection.ball_detection as ball_detection
import cv2

# detector = apriltag.Detector()


if __name__ == "__main__":
    url_AT = 'http://10.0.0.3/cam-hi.jpg' #2
    url_gb = 'http://10.0.0.6/cam-hi.jpg' #4


    tid,tx,ty,tz,rx,ry,rz = 0,0,0,0,0,0,0

    # GB
    ball_detection.modifyCore()
    # modelLoc          = './ball_detection/model_weights/'
    # modelFile         = 'model_weights-2-20210818_002355-cpu.pth'
    model = ball_detection.returnModel(device, labelSet, modelLoc, modelFile)

    gbx, gby  = -1, -1   # by default (-1 means no found green ball)
    gb_dist = -1         # by default (-1 means no found green ball)


    while True:
        tid,tx,ty,tz,rx,ry,rz = get_AT_6DOF_info(url_AT)

        # gbx, gby, gb_dist = ball_detection.detectLive(url_gb, model, minDetectionScore, showSight = True)

        key=cv2.waitKey(5)
        if key==ord('q'):
            break

    cv2.destroyAllWindows()
