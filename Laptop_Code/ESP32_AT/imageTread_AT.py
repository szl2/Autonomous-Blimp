import cv2
from urllib.request import urlopen, Request
import numpy as np
import time
import math
from math import sin,atan2,cos,pi,atan,sqrt
# import apriltag
import sys
from pupil_apriltags import Detector
def nothing(x):
    pass


# detector = apriltag.Detector()

detector = Detector()

def get_AT_6DOF_info(url):
    tid,tx,ty,tz= 0,100000,100000,0
    rx = [0.0,0.0,0.0]
    ry = [0.0, 0.0, 0.0]
    rz = [0.0, 0.0, 0.0]
    pose_r = np.array([[0,0,0],[0,0,0],[0,0,0]])
    header = {
        "User-Agent": "Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/92.0.4515.159 Safari/537.36."}
    req = Request(url, headers=header)
    img_resp = urlopen(req, timeout=60)
    imgnp = np.array(bytearray(img_resp.read()), dtype=np.uint8)
    frame = cv2.imdecode(imgnp, -1)

    # ret,frame = cap.read()
    gray_image = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    h, w, _ = frame.shape
    # put a dot in center of the frame
    cv2.circle(frame, (w // 2, h // 2), 7, (255, 0, 0), -1)
    # set camera parameters
    fx = 800
    fy = 600
    cx = 400
    cy = 300
    #AT_size = 0.16
    AT_size = 0.74
    results = detector.detect(gray_image, estimate_tag_pose=True, camera_params=[fx, fy, cx, cy], tag_size=AT_size)
    debug_print = 0
    for r in results:
        # extract the bounding box (x, y)-coordinates for the AprilTag
        # and convert each of the (x, y)-coordinate pairs to integers
        (ptA, ptB, ptC, ptD) = r.corners
        ptB = (int(ptB[0]), int(ptB[1]))
        ptC = (int(ptC[0]), int(ptC[1]))
        ptD = (int(ptD[0]), int(ptD[1]))
        ptA = (int(ptA[0]), int(ptA[1]))
        tx, ty, tz = r.pose_t # in meters
        rx,ry,rz = r.pose_R
        pose_r = r.pose_R
        tid = r.tag_id

        # draw the bounding box of the AprilTag detection
        cv2.line(frame, ptA, ptB, (0, 255, 0), 5)
        cv2.line(frame, ptB, ptC, (0, 255, 0), 5)
        cv2.line(frame, ptC, ptD, (0, 255, 0), 5)
        cv2.line(frame, ptD, ptA, (0, 255, 0), 5)
        # draw the center (x, y)-coordinates of the AprilTag
        (cX, cY) = (int(r.center[0]), int(r.center[1]))
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
    cv2.imshow("Image", frame)

    return tid,tx,ty,tz,rx,ry,rz

def get_AT_6DOF_info_posR(url):
    tid,tx,ty,tz= 0,100000,100000,0
    rx = [0.0,0.0,0.0]
    ry = [0.0, 0.0, 0.0]
    rz = [0.0, 0.0, 0.0]
    pose_r = np.array([[0,0,0],[0,0,0],[0,0,0]])
    header = {
        "User-Agent": "Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/92.0.4515.159 Safari/537.36."}
    req = Request(url, headers=header)
    img_resp = urlopen(req, timeout=60)
    imgnp = np.array(bytearray(img_resp.read()), dtype=np.uint8)
    frame = cv2.imdecode(imgnp, -1)

    # ret,frame = cap.read()
    gray_image = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    h, w, _ = frame.shape
    # put a dot in center of the frame
    cv2.circle(frame, (w // 2, h // 2), 7, (255, 0, 0), -1)
    # set camera parameters
    fx = 800
    fy = 600
    cx = 400
    cy = 300
    #AT_size = 0.16
    AT_size = 0.74
    results = detector.detect(gray_image, estimate_tag_pose=True, camera_params=[fx, fy, cx, cy], tag_size=AT_size)
    debug_print = 0
    for r in results:
        # extract the bounding box (x, y)-coordinates for the AprilTag
        # and convert each of the (x, y)-coordinate pairs to integers
        (ptA, ptB, ptC, ptD) = r.corners
        ptB = (int(ptB[0]), int(ptB[1]))
        ptC = (int(ptC[0]), int(ptC[1]))
        ptD = (int(ptD[0]), int(ptD[1]))
        ptA = (int(ptA[0]), int(ptA[1]))
        tx, ty, tz = r.pose_t # in meters
        rx,ry,rz = r.pose_R
        pose_r = r.pose_R
        tid = r.tag_id

        # draw the bounding box of the AprilTag detection
        cv2.line(frame, ptA, ptB, (0, 255, 0), 5)
        cv2.line(frame, ptB, ptC, (0, 255, 0), 5)
        cv2.line(frame, ptC, ptD, (0, 255, 0), 5)
        cv2.line(frame, ptD, ptA, (0, 255, 0), 5)
        # draw the center (x, y)-coordinates of the AprilTag
        (cX, cY) = (int(r.center[0]), int(r.center[1]))
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
    cv2.imshow("Image", frame)

    return tid,tx,ty,tz,rx,ry,rz,pose_r


def rotationM2Euler(rx,ry,rz) :
    R_00 = rx[0]
    R_01 = rx[1]
    R_02 = rx[2]
    R_10 = ry[0]
    R_11 = ry[1]
    R_12 = ry[2]
    R_20 = rz[0]
    R_21 = rz[1]
    R_22 = rz[2]

    sy = math.sqrt(R_00 * R_00 + R_10 * R_10)
    singular = sy < 1e-6
    if not singular:
        x = math.atan2(R_21, R_22)
        y = math.atan2(-R_20, sy)
        z = math.atan2(R_10, R_00)
    else:
        x = math.atan2(-R_12, R_11)
        y = math.atan2(-R_20, sy)
        z = 0

    print("phi =", np.rad2deg(x))
    print("theta  =", np.rad2deg(y))
    print("psi =", np.rad2deg(z))

    """
    tol = sys.float_info.epsilon * 10

    if abs(R_00) < tol and abs(R_10) < tol:
        eul1 = 0
        eul2 = atan2(-R_20, R_00)
        eul3 = atan2(-R_12, R_11)
    else:
        eul1 = atan2(R_10, R_00)
        sp = sin(eul1)
        cp = cos(eul1)
        eul2 = atan2(-R_20, cp * R_00 + sp * R_10)
        eul3 = atan2(sp * R_02 - cp * R_12, cp * R_11 - sp * R_01)

    print("phi =", np.rad2deg(eul1))
    print("theta  =", np.rad2deg(eul2))
    print("psi =", np.rad2deg(eul3))
    """


def rotationMatrixToEulerAngles(R) :
    assert(isRotationMatrix(R))
    sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
    singular = sy < 1e-6
    if  not singular :
        x = math.atan2(R[2,1] , R[2,2])
        y = math.atan2(-R[2,0], sy)
        z = math.atan2(R[1,0], R[0,0])
    else :
        x = math.atan2(-R[1,2], R[1,1])
        y = math.atan2(-R[2,0], sy)
        z = 0
    print("R_phi =", np.rad2deg(x))
    print("R_theta  =", np.rad2deg(y))
    print("R_psi =", np.rad2deg(z))

def R2EA(pose_r):
    r11 = pose_r[0][0]
    r12 = pose_r[0][1]
    r13 = pose_r[0][2]
    r21 = pose_r[1][0]
    r22 = pose_r[1][1]
    r23 = pose_r[1][2]
    r31 = pose_r[2][0]
    r32 = pose_r[2][1]
    r33 = pose_r[2][2]

    AprilTagPitch = round(np.degrees(atan(-r31 / sqrt((r32 * r32) + (r33 * r33)))), 3)
    AprilTagRoll = round(np.degrees(atan(-r32 / r33)), 3)
    ApriTagYaw = round(np.degrees(atan(r21 / r11)), 3)

    print("pitch =", AprilTagPitch)
    print("roll  =", AprilTagRoll)
    print("yaw =", ApriTagYaw )

if __name__ == "__main__":
    #change the IP address below according to the
    #IP shown in the Serial monitor of Arduino code
    # url='http://192.168.4.1/cam-hi.jpg'
    # url='http://192.168.1.107/cam-hi.jpg'
    url='http://192.168.4.1/cam-hi.jpg'
    url = 'http://192.168.1.118/cam-hi.jpg'
    url = 'http://10.0.0.3/cam-hi.jpg'  #03
    url = 'http://10.0.0.5/cam-hi.jpg'  #01
    url = 'http://10.0.0.6/cam-hi.jpg'  #02
    url = 'http://10.0.0.9/cam-hi.jpg'  # 6


    # cv2.namedWindow("live transmission", cv2.WINDOW_AUTOSIZE)

    # cv2.namedWindow("live transmission", cv2.WINDOW_NORMAL)

    detector = Detector()
    tid,tx,ty,tz,rx,ry,rz = 0,0,0,0,0,0,0
    test_webcam = 0
    if test_webcam == 1:
        cap = cv2.VideoCapture(0)
    while True:

        tid,tx,ty,tz,rx,ry,rz,R = get_AT_6DOF_info(url)

        print("testing new function")
        print("-----------------------")

        print("tid:{}".format(tid))
        print("tx,ty,tz:{},{},{}".format(tx,ty,tz))
        print("rx,ry,rz:{},{},{}".format(rx,ry,rz))


        print("rotation matrix:{}".format(R))
        R2EA(R)

        # print("R00,R10,R20:{},{},{}".format(rx[0], ry[0], rz[0]))
        # rotationM2Euler(rx, ry, rz)
        """
        roll,pitch, yaw = rotationMatrixToEulerAngles(rx,ry,rz)
        roll = roll * 180 /math.pi
        pitch = pitch * 180 / math.pi
        yaw = yaw * 180 / math.pi
        print("roll,pitch,yaw:{},{},{}".format(roll, pitch, yaw))
        """
        # rotationMatrixToEulerAngles(R)

        key=cv2.waitKey(5)
        if key==ord('q'):
            break

    #cv2.destroyAllWindows()
