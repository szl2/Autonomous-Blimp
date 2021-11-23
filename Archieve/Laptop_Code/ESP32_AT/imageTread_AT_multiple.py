import cv2
from urllib.request import urlopen, Request
import numpy as np
import time
import math
# import apriltag
from pupil_apriltags import Detector

detector = Detector(families='tag36h11',
                    nthreads=1,
                    quad_decimate=1.0,
                    quad_sigma=0.0,
                    refine_edges=1,
                    decode_sharpening=0.25,
                    debug=0)

def nothing(x):
    pass



def get_AT_6DOF_info(url):
    tid,tx,ty,tz= 0,100000,100000,0
    rx = [0.0,0.0,0.0]
    ry = [0.0, 0.0, 0.0]
    rz = [0.0, 0.0, 0.0]

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
    AT_size = 0.16
    results = detector.detect(gray_image, estimate_tag_pose=True, camera_params=[fx, fy, cx, cy], tag_size=AT_size)
    debug_print = 0

    Every_April_Tag_Infos = []

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

        Single_April_Tag_Info = [tid,tx,ty,tz,rx,ry,rz]
        Every_April_Tag_Infos.append(Single_April_Tag_Info)

    # show the output image after AprilTag detection
    cv2.imshow("Image", frame)

    return Every_April_Tag_Infos


def rotationMatrixToEulerAngles(rx,ry,rz) :

    sy = math.sqrt(rx[0] * rx[0] + ry[0] * ry[0])
    roll = math.atan2(rz[1] , rz[2])
    pitch = math.atan2(-rz[0], sy)
    yaw = math.atan2(ry[0], rx[0])

    return np.array([roll, pitch, yaw])

def AT_info_list(Every_April_Tag_Infos):
    num_of_ATs = len(Every_April_Tag_Infos)

    tids = []
    txs,tys,tzs = [],[],[]
    rxs,rys,rzs = [],[],[]

    if num_of_ATs != 0:
        for i in range(num_of_ATs):
            if i == 0:
                print("========================")
                print("=== Num of AT: {} =======".format(num_of_ATs))
                print("========================")

            tid = Every_April_Tag_Infos[i][0]
            tx = Every_April_Tag_Infos[i][1]
            ty = Every_April_Tag_Infos[i][2]
            tz = Every_April_Tag_Infos[i][3]
            rx = Every_April_Tag_Infos[i][4]
            ry = Every_April_Tag_Infos[i][5]
            rz = Every_April_Tag_Infos[i][6]

            tids.append(tid)
            txs.append(tx)
            tys.append(ty)
            tzs.append(tz)
            rxs.append(rx)
            rys.append(ry)
            rzs.append(rz)
    return tids,txs,tys,tzs,rxs,rys,rzs

def get_AT_6DOF_info_list(url):
    Every_April_Tag_Infos = get_AT_6DOF_info(url)
    ids,txs,tys,tzs,rxs,rys,rzs = AT_info_list(Every_April_Tag_Infos)
    return ids,txs,tys,tzs,rxs,rys,rzs

if __name__ == "__main__":
    #change the IP address below according to the
    #IP shown in the Serial monitor of Arduino code
    # url='http://192.168.4.1/cam-hi.jpg'
    # url='http://192.168.1.107/cam-hi.jpg'
    # url='http://192.168.4.1/cam-hi.jpg'
    # url = 'http://192.168.1.118/cam-hi.jpg'
    # url = 'http://192.168.0.176/cam-hi.jpg' # 2
    # url = 'http://192.168.0.112/cam-hi.jpg' # 4
    url = "http://10.0.0.3/cam-hi.jpg"

    tid,tx,ty,tz,rx,ry,rz = 0,0,0,0,0,0,0
    test_webcam = 0
    if test_webcam == 1:
        cap = cv2.VideoCapture(0)
    while True:
        # Every_April_Tag_Infos = get_AT_6DOF_info(url)
        # ids,txs,tys,tzs,rxs,rys,rzs = AT_info_list(Every_April_Tag_Infos)

        ids, txs, tys, tzs, rxs, rys, rzs = get_AT_6DOF_info_list(url)
        print("ids:{}".format(ids))
        print("txs:{}".format(txs))
        print("tys:{}".format(tys))
        print("tzs:{}".format(tzs))
        print("rxs:{}".format(rxs))
        print("rys:{}".format(rys))
        print("rzs:{}".format(rzs))
        """
        num_of_ATs = len(Every_April_Tag_Infos)

        for i in range(num_of_ATs):
            if i == 0:
                print("========================")
                print("=== Num of AT: {} =======".format(num_of_ATs) )
                print("========================")

            tid = Every_April_Tag_Infos[i][0]
            tx  = Every_April_Tag_Infos[i][1]
            ty  = Every_April_Tag_Infos[i][2]
            tz  = Every_April_Tag_Infos[i][3]
            rx  = Every_April_Tag_Infos[i][4]
            ry  = Every_April_Tag_Infos[i][5]
            rz  = Every_April_Tag_Infos[i][6]

            print("-----------------------")
            print("tid:{}".format(tid))
            print("tx,ty,tz:{},{},{}".format(tx,ty,tz))
            print("rx,ry,rz:{},{},{}".format(rx,ry,rz))

            # R = np.array([rx,ry,rz])
            roll,pitch, yaw = rotationMatrixToEulerAngles(rx,ry,rz)
            roll = roll * 180 /math.pi
            pitch = pitch * 180 / math.pi
            yaw = yaw * 180 / math.pi
            print("roll,pitch,yaw:{},{},{}".format(roll, pitch, yaw))
        """
        key=cv2.waitKey(5)
        if key==ord('q'):
            break

    cv2.destroyAllWindows()


