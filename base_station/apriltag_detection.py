import cv2
import time
import math
import numpy as np
from data_logging import *
from pupil_apriltags import Detector
from urllib.request import urlopen, Request

detector = Detector()

def get_AT_6DOF_info(url, logger, detectFlag = True):
    """
    If you also want to extract the tag pose, estimate_tag_pose should be set to True
    and camera_params ([fx, fy, cx, cy])
    and tag_size (in meters) should be supplied.
    The detect method returns a list of Detection objects each having
    the following attributes
    (note that the ones with an asterisks are computed only if estimate_tag_pose=True):
    """
    """
    So fx and fy are the focal lengths expressed in pixels.
    Cx and Cy describe the coordinates of the so called principal
    point that should be in the center of the image.
    It is e.g. not in the center of the image if you cropped the image,
    what you should never do when calibrating.
    fx, fy, cx, cy are given in Pixels in Computer Vision ( and openCV)
    but e.g. in Photogrammetry you often use mm
    """
    tid,tx,ty,tz = 0,100000,100000,0
    rx = [0.0, 0.0, 0.0]
    ry = [0.0, 0.0, 0.0]
    rz = [0.0, 0.0, 0.0]
    blank_frame = np.zeros((400,300,3), np.uint8)

    header = {"User-Agent": "Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/92.0.4515.159 Safari/537.36."}

    req = Request(url, headers = header)
    try:
        img_resp = urlopen(req, timeout = 0.5)
        imgnp = np.array(bytearray(img_resp.read()), dtype = np.uint8)
    except:
        print ("April Tag  Camera NOT RUNNING")
        log_variable(logger, 'April Tag Camera NOT RUNNING')
        return tid,tx,ty,tz,rx,ry,rz,blank_frame
    frame = cv2.imdecode(imgnp, -1)

    gray_image = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    h, w, _ = frame.shape

    if not detectFlag:
        resized_frame = cv2.resize(frame,(400,300))
        cv2.imshow("Image", resized_frame)
        return tid,tx,ty,tz,rx,ry,rz,resized_frame

    # put a dot in center of the frame
    cv2.circle(frame, (w // 2, h // 2), 7, (255, 0, 0), -1)
    # set camera parameters
    fx = 800
    fy = 600
    cx = 400
    cy = 300
    #AT_size = 0.16
    AT_size = 0.74 #SIZE NEED TO BE FIXED
    results = detector.detect(gray_image, estimate_tag_pose=True, camera_params=[fx, fy, cx, cy], tag_size=AT_size)
    debug_print = 0

    #Closest_tz = 10000

    log_variable(logger, 'April Tag results', results)
    for r in results:
        closest_tag_dist = 10000
        tid = r.tag_id
        leftSideAT = [1,2,3,4,5]
        print("Detected TID: ", tid)
        if tid in leftSideAT:
            ttx, tty, ttz = r.pose_t
            if (ttz < closest_tag_dist):
                closest_tag_dist = ttz
                tx, ty, tz = r.pose_t # in meters
                rx, ry, rz = r.pose_R

                # if tid == 6:
                #     tx, ty = -tx, -ty

                # extract the bounding box (x, y)-coordinates for the AprilTag
                # and convert each of the (x, y)-coordinate pairs to integers
                (ptA, ptB, ptC, ptD) = r.corners
                ptB = (int(ptB[0]), int(ptB[1]))
                ptC = (int(ptC[0]), int(ptC[1]))
                ptD = (int(ptD[0]), int(ptD[1]))
                ptA = (int(ptA[0]), int(ptA[1]))
                log_variable(logger, '[ptA, ptB, ptC, ptD]', [ptA, ptB, ptC, ptD])

                # draw the bounding box of the AprilTag detection
                cv2.line(frame, ptA, ptB, (0, 255, 0), 5)
                cv2.line(frame, ptB, ptC, (0, 255, 0), 5)
                cv2.line(frame, ptC, ptD, (0, 255, 0), 5)
                cv2.line(frame, ptD, ptA, (0, 255, 0), 5)

                # draw the center (x, y)-coordinates of the AprilTag
                (cX, cY) = (int(r.center[0]), int(r.center[1]))
                log_variable(logger, '[cX, cY]', [cX, cY])
                cv2.circle(frame, (cX, cY), 5, (0, 0, 255), -1)

                # draw the tag family on the image
                tagFamily = r.tag_family.decode("utf-8")

                cv2.putText(frame, tagFamily, (ptA[0], ptA[1] - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                cv2.putText(frame, "tx: {:.2f}  ty: {:.2f}  tz:{:.2f}".format(tx[0], ty[0], tz[0]), (ptA[0], ptA[1] + 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                if debug_print == 1:
                    print("tx,ty,tz:{},{},{}".format(tx, ty, tz))
                    print("cX,cY:{},{}".format(cX, cY))
                    print("[INFO] tag id: {}".format(tid))

    # show the output image after AprilTag detection
    resized_frame = cv2.resize(frame,(400,300))
    cv2.imshow("Image", resized_frame)
    return tid,tx,ty,tz,rx,ry,rz,resized_frame


def rotationMatrixToEulerAngles(rx,ry,rz) :
    sy = math.sqrt(rx[0] * rx[0] + ry[0] * ry[0])
    roll = math.atan2(rz[1] , rz[2])
    pitch = math.atan2(-rz[0], sy)
    yaw = math.atan2(ry[0], rx[0])
    return np.array([roll, pitch, yaw])


if __name__ == "__main__":
    #change the IP address below according to that shown in the Serial monitor of Arduino code
    url = 'http://10.0.0.7/cam-hi.jpg'  #06

    # cv2.namedWindow("live transmission", cv2.WINDOW_AUTOSIZE)
    # cv2.namedWindow("live transmission", cv2.WINDOW_NORMAL)

    detector = Detector()
    tid,tx,ty,tz,rx,ry,rz = 0,0,0,0,0,0,0
    test_webcam = 0
    if test_webcam == 1:
        cap = cv2.VideoCapture(0)
    while True:
        tid,tx,ty,tz,rx,ry,rz = get_AT_6DOF_info(url)

        print("tid:{}".format(tid))
        print("tx,ty,tz:{},{},{}".format(tx,ty,tz))
        print("rx,ry,rz:{},{},{}".format(rx,ry,rz))

        # R = np.array([rx,ry,rz])
        roll, pitch, yaw = rotationMatrixToEulerAngles(rx,ry,rz)
        roll = roll * 180 /math.pi
        pitch = pitch * 180 / math.pi
        yaw = yaw * 180 / math.pi
        print("roll,pitch,yaw:{},{},{}".format(roll, pitch, yaw))

        key = cv2.waitKey(5)
        if key == ord('q'):
            break

    #cv2.destroyAllWindows()
