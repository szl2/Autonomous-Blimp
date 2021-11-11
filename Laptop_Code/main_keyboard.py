import time
import serial

import simple_pid.PID as PID
import timeit
import pygame
from constants import *
from ESP32_AT.imageTread_AT import get_AT_6DOF_info

global ml,esp_cam_on,openmv_on
ml = 1
esp_cam_on = 1
openmv_on = 0
seekVertDir = 1
AT_detected_time = time.time()
if ml == 1:
    import ball_detection.ball_detection as ball_detection

# ========= Serial Port I/O ===========

def serial_port_in_v1(serial_port):
    '''
    Description:
        Take all ESP32_Master serial port's printIn and take all necessary input object

    Input:
        serial_port     :    serial.Serail object

    Output:
        tx, ty, tz, rx, ry, rz, LIDAR_dist1, LIDAR_dist2, DebugM
    '''

    # DEBUG Verbose
    print("initiating one round of serial in ...")

    for i in range(7):
        line = serial_port.readline()
        val = int(line.decode())

        if   i == 0:
            tx = val
        elif i == 1:
            ty = val
        elif i == 2:
            tz = val
        elif i == 3:
            rx = val
        elif i == 4:
            ry = val
        elif i == 5:
            rz = val
        elif i == 6:
            LIDAR_dist1 = val
        # elif i == 7:
            # LIDAR_dist2 = val

    LIDAR_dist2 = 0
    line = serial_port.readline()
    debugM = line.decode()

    # DEBUG Verbose
    print("tx:{}".format(tx))
    print("ty:{}".format(ty))
    print("tz:{}".format(tz))
    print("rx:{}".format(rx))
    print("ry:{}".format(ry))
    print("rz:{}".format(rz))
    print("LIDAR_dist1:{}".format(LIDAR_dist1))
    print("LIDAR_dist2:{}".format(LIDAR_dist2))
    print(debugM)

    return tx, ty, tz, rx, ry, rz, LIDAR_dist1, LIDAR_dist2, debugM

def serial_port_in_v2(serial_port):
    '''
    Description:
        Take all ESP32_Master serial port's printIn and take all necessary input object

    Input:
        serial_port     :    serial.Serail object

    Output:
         LIDAR_dist1, LIDAR_dist2, DebugM
    '''
    # debug info

    print("initiating serial in V2...")
    for i in range(3):
        line = serial_port.readline()

        if  i == 0:
            val = int(line.decode())
            LIDAR_dist1 = val
        elif i == 1:
            val = int(line.decode())
            LIDAR_dist2 = val
        elif i == 2:
            debugM = line.decode()

    print("LIDAR_dist1:{}".format(LIDAR_dist1))
    print("LIDAR_dist2:{}".format(LIDAR_dist2))
    print(debugM)

    return LIDAR_dist1, LIDAR_dist2, debugM

def serial_port_out(serial_port, pwm1, pwm2, pwm3, pwm4, dir1, dir2, dir3, dir4):
    '''
    Description:
        Feed to ESP32_Master to send ESP32_Slave necessary information
        the format of sending is pwm are 3 digit space

    Input:
        serial_port                                     :   serial.Serail object
        pwm1, pwm2, pwm3, pwm4, dir1, dir2, dir3, dir4  :   variables to send

    Output:
        None
    '''
    output_message = ''

    for pwm_itr in [pwm1, pwm2, pwm3, pwm4]:
        # print(pwm_itr)
        if len(str(pwm_itr)) == 2:
            output_message += '0'
        elif len(str(pwm_itr)) == 1:
            output_message += '00'
        output_message += str(pwm_itr)
        print(pwm_itr)

    output_message = output_message + dir1 + dir2 +  dir3 +  dir4 + '\n'
    print("serial out ...")
    print(output_message)
    serial_port.write(output_message.encode())


# ====== Logic-directing Functions ====
def ball_detect(gbx, gby):
    '''
    return True if green ball is detected
    '''
    if gbx == -1 and gby == -1:
        return False
    else:
        return True

def goal_detect(tx,ty):
    '''
    return True if April Tag is detected
    '''
    if tx == 100000 and ty == 100000:
        return False
    else:
        return True

def ball_capture(LIDAR_dist1):
    '''
    return True if April Tag is detected
    '''
    if (LIDAR_dist1 < LIDAR_Thres) and (LIDAR_dist1 > 0):  # Ball captured
        return True
    else:
        return False

def AT_detect(tx,ty):
    if tx == 100000 and ty == 100000:
        return False
    else:
        return True

def stop_all():
    pwm1, pwm2, pwm3, pwm4 = 0 , 0 , 0 , 0
    dir1, dir2, dir3, dir4 = '+', '-', '+', '-'
    return pwm1, pwm2, pwm3, pwm4, dir1, dir2, dir3, dir4

def move2goal(tx, ty,tz):
    """
    Description:
        Given the center of the AT tx, ty. Call PID control to output the blimp
        motor to manuver to the goal

    Input:
        tx    :    x component, center of April Tag
        ty    :    y component, center of Aprol Tag

    Output:
        pwm1, pwm2, pwm3, pwm4
        dir1, dir2, dir3, dir4
    """



    # April Tag Center
    setpoint_x1 = 0.0
    setpoint_y1 = 0.0

    if tz < 2.0:
        kpy_g,kiy_g,kdy_g =  2, 0.1, 0.5
        base_speed = 200
        AT_goal_Delta = -150
    else:
        kpy_g,kiy_g,kdy_g = 2, 0.1, 0.5
        # kpy_g,kiy_g,kdy_g = 0.0, 0.00, 0.0
        base_speed = 140
        AT_goal_Delta = 0
    inputx = tx * 100/ 1.00
    inputy =  (ty * 100 + AT_goal_Delta) / 1.00 #

    pid_x = PID(kpx_g, kix_g, kdx_g, setpoint = setpoint_x1)
    pid_y = PID(kpy_g, kiy_g, kdy_g, setpoint = setpoint_y1)

    pid_x.auto_mode = True
    pid_x.set_auto_mode(True, last_output = 8.0)
    pid_x.output_limits = (-255,255)
    pid_y.output_limits = (-255,255)

    outputx = pid_x(inputx)
    outputy = pid_y(inputy)
    print("outputy:{}".format(outputy))
    # Vertical
    pwm1 = abs(outputy)
    pwm2 = abs(outputy)

    if(outputy > 0):
        dir1 = '+'
        dir2 = '+'
    else:
        dir1 = '-'
        dir2 = '-'

    # Horizontal
    lspeed = -1 * outputx + base_speed
    rspeed =  1 * outputx + base_speed
    pwm3 = abs(lspeed)
    pwm4 = abs(rspeed)
    if (lspeed > 0):
        dir3 = '+'
    else:
        dir3 = '-'
    if (rspeed > 0):
        dir4 = '+'
    else:
        dir4 = '-'

    return int(pwm1), int(pwm2), int(pwm3), int(pwm4), dir1, dir2, dir3, dir4


def ball_seeking(count_h,tx,ty):
    """
    Description:
        By default, when there ball is not determined capture, the manuver of the
        motors to have it scan its surronding 360 degrees

    Input:
        none

    Output:
        pwm1, pwm2, pwm3, pwm4
        dir1, dir2, dir3, dir4
    """
    global set_h, seekVertDir
    if openmv_on == 1:
        delta_h = 100 # cm
        threshold_h = 20
    elif esp_cam_on == 1:
        delta_h = 2 # meter
        threshold_h = 0.3

    AT_detected = AT_detect(tx, ty)

    # count_h = 0 #
    if AT_detected:
        current_h = get_altitude_from_AT(AT_h1,ty)

        if count_h == 0:
            set_h = current_h + delta_h  # in meter
            count_h = 1  # moving up

        # pwm1, pwm2, pwm3, pwm4, dir1, dir2, dir3, dir4 = vertical_control(current_h, set_h)
        diff_h = abs(set_h - current_h)
        if diff_h < threshold_h:
            new_ssp = 200
            pwm1, pwm2, pwm3, pwm4, dir1, dir2, dir3, dir4 = rotate_one_direction_ssp(new_ssp)
            count_h = 0
        else:
            pwm1, pwm2, pwm3, pwm4, dir1, dir2, dir3, dir4 = vertical_control(current_h, set_h)
    else:
        pwm1, pwm2, pwm3, pwm4, dir1, dir2, dir3, dir4 = rotate_one_direction()


    # if time.time() > AT_detected_time + AT_detectBreak:
    #     AT_detected = AT_detect(tx, ty)
    #     AT_detected_time = time.time()
    #
    # while facingWall():
    #     pwm1, pwm2, pwm3, pwm4, dir1, dir2, dir3, dir4 = rotate_one_direction_ssp(new_ssp)
    #
    # if AT_detected:
    #     current_h = get_altitude_from_AT(AT_h1,ty)
    #
    # if LIDAR_dist2 < LIDAR2_Thres:
    #     seekVertDir = 0
    # if baroHeight < baroThres:
    #     seekVertDir = 1
    # pwm1, pwm2, pwm3, pwm4, dir1, dir2, dir3, dir4 = move_in_spiral(new_ssp, seekVertDir)

    return int(pwm1), int(pwm2), int(pwm3), int(pwm4), dir1, dir2, dir3, dir4

def vertical_control(current_h,set_h):

    pid_h = PID(kph, kih, kdh, setpoint = set_h)
    pid_h.auto_mode = True
    pid_h.set_auto_mode(True, last_output = 8.0)
    pid_h.output_limits = (-255,255)

    input_h = current_h
    output_h = pid_h(input_h)

    pwm1 = abs(output_h)
    pwm2 = abs(output_h)
    pwm3 = 0
    pwm4 = 0
    dir1 = '+'
    dir2 = '+'
    dir3 = '+'
    dir4 = '+'

    return int(pwm1), int(pwm2), int(pwm3), int(pwm4), dir1, dir2, dir3, dir4

def rotate_one_direction():
    pwm1, pwm2, pwm3, pwm4 = 0, 0, seeking_speed, seeking_speed
    dir1, dir2, dir3, dir4 = '+', '+', '+', '-'

    return int(pwm1), int(pwm2), int(pwm3), int(pwm4), dir1, dir2, dir3, dir4

def rotate_one_direction_ssp(ssp):
    pwm1, pwm2, pwm3, pwm4 = 0, 0, ssp, ssp
    dir1, dir2, dir3, dir4 = '+', '+', '+', '-'

    return int(pwm1), int(pwm2), int(pwm3), int(pwm4), dir1, dir2, dir3, dir4

def move_in_spiral(ssp, dir):
    pwm1, pwm2, pwm3, pwm4 = ssp, ssp, ssp, ssp/2
    dir1, dir2, dir3, dir4 = '+', '+', '+', '-'
    if dir == 0:
        dir1, dir2 = '-', '-'

    return int(pwm1), int(pwm2), int(pwm3), int(pwm4), dir1, dir2, dir3, dir4

def move2ball(gbx, gby, gb_dist):
    """
    Description:
        Given the center of x y dist of green ball detected. Call PID control to
        output the blimp motor to manuver to the green ball

    Input:
        gbx     :  x component, center of green ball
        gby     :  y component, center of green ball
        gb_dist :  distance to green ball

    Output:
        pwm1, pwm2, pwm3, pwm4
        dir1, dir2, dir3, dir4
    """
    inputx = gbx / 1.00
    inputy = gby / 1.00

    # ESP-Cam Center
    setpoint_x = 400
    setpoint_y = 300

    pid_x = PID(kpx, kix, kdx, setpoint = setpoint_x)
    pid_y = PID(kpy, kiy, kdy, setpoint = setpoint_y)

    pid_x.auto_mode = True
    pid_x.set_auto_mode(True, last_output = 8.0)
    pid_x.output_limits = (-255,255)
    pid_y.output_limits = (-255,255)

    outputx = pid_x(inputx)
    outputy = pid_y(inputy)

    # Vertical
    pwm1 = abs(outputy)
    pwm2 = abs(outputy)

    if(outputy > 0):
        dir1 = '+'
        dir2 = '+'
    else:
        dir1 = '-'
        dir2 = '-'

    # Horizontal
    lspeed = -1 * outputx + base_speed
    rspeed =  1 * outputx + base_speed
    pwm3 = min(abs(lspeed), 255)
    pwm4 = min(abs(rspeed), 255)
    if (lspeed > 0):
        dir3 = '+'
    else:
        dir3 = '-'
    if (rspeed > 0):
        dir4 = '+'
    else:
        dir4 = '-'

    return int(pwm1), int(pwm2), int(pwm3), int(pwm4), dir1, dir2, dir3, dir4


#  =========== Main Control ===========
def main_control(gbx, gby, gb_dist, tx, ty, tz, rx, ry, rz, LIDAR_dist1, LIDAR_dist2, debugM, count_h,bcap_man):
    '''
    Description:
        Given green ball information and AT information, the main control logic
        to manuver the blimp motors

    Input:
        gbx, gby, gb_dist                   :   green ball information
        tx, ty, tz, rx, ry, rz,             :   AirTag information
        LIDAR_dist1, LIDAR_dist2            :   Lidar info
        debugM                              :   Debug Message

    Output:
        pwm1, pwm2, pwm3, pwm4, dir1, dir2, dir3, dir4  :   Blimp motor manuver parameters
    '''
    print('in main_control')
    # placeholder
    pwm1, pwm2, pwm3, pwm4 = 0 , 0 , 0 , 0
    dir1, dir2, dir3, dir4 = '+', '-', '+', '-'

    ballDetect  = ball_detect(gbx, gby)
    # ballCapture = ball_capture(LIDAR_dist1)
    goalDetect  = goal_detect(tx, ty)

    ballCapture = 0

    # debug
    if bcap_man == 1:
        ballCapture = 1  # Manually determine Ball captured
    elif bcap_man == 0:
        ballCapture = 0 # Ball not captured
    elif bcap_man == -1:
        ballCapture = ball_capture(LIDAR_dist1)

        # goalDetect = 0
    # ballDetect = 0
    if ballCapture: # Ball captured
        print('ballCapture TRUE')
        if goalDetect:  # Goal detected
            # stop_all()  # Debug
            pwm1, pwm2, pwm3, pwm4, dir1, dir2, dir3, dir4 = move2goal(tx, ty,tz)
        else:  # Goal not detected
            # stop_all()  # Debug
            pwm1, pwm2, pwm3, pwm4, dir1, dir2, dir3, dir4 = rotate_one_direction()
    else:  # Ball not captured
        print('ballCapture FALSE')
        if ballDetect:  # Ball detected
            pwm1, pwm2, pwm3, pwm4, dir1, dir2, dir3, dir4 = move2ball(gbx,gby,gb_dist)
        else:  # Ball not detected
            # stop_all()  # Debug
            pwm1, pwm2, pwm3, pwm4, dir1, dir2, dir3, dir4 = rotate_one_direction()

            # pwm1, pwm2, pwm3, pwm4, dir1, dir2, dir3, dir4 = ball_seeking(count_h,tx,ty)

    return pwm1, pwm2, pwm3, pwm4, dir1, dir2, dir3, dir4

def auto_control(serial_port,gbx, gby, gb_dist, tx, ty, tz, rx, ry, rz, LIDAR_dist1, LIDAR_dist2, debugM,count_h,bcap_man):
    # ===== STEP 1: TAKE ALL INPUT =====
    # print('in auto_control')
    if ml ==1:
        gbx, gby, gb_dist = ball_detection.detectLive(url_gb, model, minDetectionScore, showSight = True)
    line = serial_port.readline()
    # print('auto')
    if openmv_on == 1:
        if line == b'SERIAL_IN_START\r\n':
            tx, ty, tz, rx, ry, rz, LIDAR_dist1, LIDAR_dist2, debugM = serial_port_in_v1(serial_port)
            print("gbx,gby:{},{}".format(gbx, gby))
            time.sleep(waitTime)


    if esp_cam_on == 1:
        url = 'http://10.0.0.5/cam-hi.jpg'  # 1
        tid, tx, ty, tz, rx, ry, rz = get_AT_6DOF_info(url)
        LIDAR_dist1 = 0
        LIDAR_dist2 = 0
        debugM = "using two esp32 cam"


    # ===== STEP 2: MAIN CONTROL LOOP =====
    pwm1, pwm2, pwm3, pwm4, dir1, dir2, dir3, dir4 = main_control(gbx, gby, gb_dist, tx, ty, tz, rx, ry, rz, LIDAR_dist1, LIDAR_dist2, debugM, count_h,bcap_man)

    # ===== STEP 3: FEED ALL OUTPUT =====
    serial_port_out(serial_port, pwm1, pwm2, pwm3, pwm4, dir1, dir2, dir3, dir4)

    time.sleep(waitTime)

def auto_init(init_count,gbx, gby, gb_dist, tx, ty, tz, rx, ry, rz, LIDAR_dist1, LIDAR_dist2, debugM):
    count_h = 0
    bcap_man = -1
    # gbx, gby, gb_dist = ball_detection.detectLive(url_gb, model, minDetectionScore, showSight = True)
    pwm1, pwm2, pwm3, pwm4, dir1, dir2, dir3, dir4 = main_control(gbx, gby, gb_dist, tx, ty, tz, rx, ry, rz, LIDAR_dist1, LIDAR_dist2, debugM, count_h,bcap_man)
    serial_port_out(serial_port, pwm1, pwm2, pwm3, pwm4, dir1, dir2, dir3, dir4)
    init_count += 1

    return init_count,count_h,bcap_man

def init():
    pygame.init()
    win= pygame.display.set_mode((200,200))

def keyboard_stop(flag_s,print_count_s):

    if get_key('q'):
        flag_s = 0
        print_count_s = 1
    return flag_s,print_count_s

def get_key(keyname):
    ans = False
    for eve in pygame.event.get(): pass
    keyInput = pygame.key.get_pressed()
    myKey = getattr(pygame,'K_{}'.format(keyname))

    if keyInput[myKey]:
        ans = True
    pygame.display.update()
    return ans

def stop_all():
    pwm1, pwm2, pwm3, pwm4 = 0 , 0 , 0 , 0
    dir1, dir2, dir3, dir4 = '+', '-', '+', '-'
    return pwm1, pwm2, pwm3, pwm4, dir1, dir2, dir3, dir4

def dynamic_variable(str_name_v):

    global start_speed
    global kpx,kix,kdx
    global kpy,kiy,kdy
    global kpx_g,kix_g,kdx_g
    global kpy_g,kiy_g,kdy_g

    if str_name_v == "kpx":
        kpx = input("Enter your value: ")
        print("kpx:{}".format(kpx))
    elif str_name_v == "kix":
        kix = input("Enter your value: ")
        print("kix:{}".format(kix))
    elif str_name_v == "kdx":
        kdx = input("Enter your value: ")
        print("kdx:{}".format(kdx))
    elif str_name_v == "stsp":
        start_speed = input("Enter your value: ")
        print("start_speed:{}".format(start_speed))
    elif str_name_v == "kpy":
        kpx = input("Enter your value: ")
        print("kpy:{}".format(kpy))
    elif str_name_v == "kiy":
        kix = input("Enter your value: ")
        print("kiy:{}".format(kiy))
    elif str_name_v == "kdy":
        kdx = input("Enter your value: ")
        print("kdy:{}".format(kdy))
    if str_name_v == "kpx":
        kpx = input("Enter your value: ")
        print("kpx:{}".format(kpx))
    elif str_name_v == "kix_g":
        kix = input("Enter your value: ")
        print("kix_g:{}".format(kix_g))
    elif str_name_v == "kdx":
        kdx = input("Enter your value: ")
        print("kdx_g:{}".format(kdx_g))
    elif str_name_v == "kpy_g":
        kpx = input("Enter your value: ")
        print("kpy_g:{}".format(kpy_g))
    elif str_name_v == "kiy_g":
        kix = input("Enter your value: ")
        print("kiy_g:{}".format(kiy_g))
    elif str_name_v == "kdy_g":
        kdx = input("Enter your value: ")
        print("kdy_g:{}".format(kdy_g))

def variables_change_once():

    str_name = input("Enter your variable: ")
    dynamic_variable(str_name)

def manual_control(Ctl_com,serial_port):

    if get_key("w"):
        val = start_speed
        Ctl_com = manual_command(val, val, 0, 0, "+", "+", "+", "+")

    elif get_key("s"):
        val = start_speed
        Ctl_com = manual_command(val, val, 0, 0, "-", "-", "+", "+")

    if get_key("UP"):
        val = start_speed

        Ctl_com = manual_command(0, 0, val, val, "+", "+", "+", "+")
    elif get_key("DOWN"):
        val = start_speed
        Ctl_com = manual_command(0, 0, val, val, "+", "+", "-", "-")

    elif get_key("LEFT"):
        val = start_speed
        Ctl_com = manual_command(0, 0, val, val, "+", "+", "-", "+")

    elif get_key("RIGHT"):
        val = start_speed
        Ctl_com = manual_command(0, 0, val, val, "+", "+", "+", "-")
    elif get_key("r"):
        val = start_speed
        Ctl_com = manual_command( val, val,0,0, "+", "-", "+", "+")

    pwm1, pwm2, pwm3, pwm4, dir1, dir2, dir3, dir4 = decode_ctl(Ctl_com)

    waitTime = 0.05
    # changed
    # gbx, gby, gb_dist = ball_detection.detectLive(url_gb, model, minDetectionScore, showSight = True)

    serial_port_out(serial_port, pwm1, pwm2, pwm3, pwm4, dir1, dir2, dir3, dir4)

    time.sleep(waitTime)

def manual_command(val1,val2,val3,val4,sign1,sign2,sign3,sign4):
    pwm1 = val1
    pwm2 = val2
    pwm3 = val3
    pwm4 = val4
    dir1 = sign1
    dir2 = sign2
    dir3 = sign3
    dir4 = sign4
    return pwm1,pwm2,pwm3,pwm4,dir1,dir2,dir3,dir4

def decode_ctl(Ctl_com):
    pwm1 = Ctl_com[0]
    pwm2 = Ctl_com[1]
    pwm3 = Ctl_com[2]
    pwm4 = Ctl_com[3]
    dir1 = Ctl_com[4]
    dir2 = Ctl_com[5]
    dir3 = Ctl_com[6]
    dir4 = Ctl_com[7]
    return pwm1, pwm2, pwm3, pwm4, dir1, dir2, dir3, dir4

def get_altitude_from_AT(AT_h,ty):
    # below AT center, ty > 0
    # above AT center, ty < 0
    altitude = AT_h - ty
    return altitude



def test_function():
    url_AT = 'http://10.0.0.5/cam-hi.jpg'  # 1
    url_gb = 'http://10.0.0.4/cam-hi.jpg'  # 6

    tid, tx, ty, tz, rx, ry, rz = get_AT_6DOF_info(url_AT)
    gbx, gby, gb_dist = ball_detection.detectLive(url_gb, model, minDetectionScore, showSight=True)

    print("testing new function")
    print("-----------------------")
    print("tid:{}".format(tid))
    print("tx,ty,tz:{},{},{}".format(tx,ty,tz))
    print("rx,ry,rz:{},{},{}".format(rx,ry,rz))
    print("gbx,gby,gb_dist:{},{},{}".format(gbx,gby,gb_dist))

def test_function1():
    url = 'http://192.168.0.230/cam-hi.jpg'
    tid, tx, ty, tz, rx, ry, rz = get_AT_6DOF_info(url)

    print("testing new function")
    print("-----------------------")
    print("tid:{}".format(tid))
    print("tx,ty,tz:{},{},{}".format(tx,ty,tz))
    print("rx,ry,rz:{},{},{}".format(rx,ry,rz))

def manual_in_auto(Ctl_com, serial_port, flag):
    if get_key('m'):
        flag = 12
    while (flag == 12):
        manual_control(Ctl_com, serial_port)
        flag = manual_return2auto('r',flag)
    return flag

def manual_return2auto(key_press,flag_r):
    if get_key(key_press):
        flag_r = 1
    return flag_r

def manual_ballcapture(bcap_man):
    if get_key('b'): # the green ball is captured
        bcap_man = 1
    elif get_key('n'): # no ball is captured
        bcap_man = 0
    elif get_key('v'): # visual
        bcap_man = -1
    return bcap_man

# ===== Main Function =====
if __name__ == '__main__':
    # =========== SET UP ============
    # Defining Variables for ESP 32 Serial I/O
    PORT = "COM11" # for Alienware
    serial_port = serial.Serial(PORT, 115200)
    serial_port.close()
    serial_port.open()

    # Weit Time
    waitTime = 0.05

    # Loading the PyTorch ML model for ball detection
    if ml == 1:
        ball_detection.modifyCore()
        model = ball_detection.returnModel(device, labelSet, modelLoc, modelFile)

    # =========== DECLARE VARIABLES ===========
    # ESP CAM In
    gbx, gby  = -1, -1   # by default (-1 means no found green ball)
    gb_dist = -1         # by default (-1 means no found green ball)

    # Serial Port In
    tx, ty, tz = 100000, 100000, 100000  # by default (0 means no found AirTag)
    rx, ry, rz = 0, 0, 0
    LIDAR_dist1 = 0
    LIDAR_dist2 = 0
    debugM = 'Testing'

    # Serial Port Out
    pwm1, pwm2, pwm3, pwm4 = 0 , 0 , 0 , 0   # Not moving
    dir1, dir2, dir3, dir4 = '+', '+', '+', '+'
    Ctl_com = [0, 0, 0, 0, "+", "+", "+", "+"]

    # Trigger the ESP32_SLAVE to talk first
    """
    gbx, gby, gb_dist = ball_detection.detectLive(url_gb, model, minDetectionScore, showSight = True)
    pwm1, pwm2, pwm3, pwm4, dir1, dir2, dir3, dir4 = main_control(gbx, gby, gb_dist, tx, ty, tz, rx, ry, rz, LIDAR_dist1, LIDAR_dist2, debugM, count_h)
    serial_port_out(serial_port, pwm1, pwm2, pwm3, pwm4, dir1, dir2, dir3, dir4)
    """
    flag = 0
    init_count = 0
    print_count = 0
    global start_speed
    start_speed = 70
    count_h = 0
    init()
    # =========== LOOP FOREVER===========
    while True:
        if get_key('a'):
            flag = 1
            while (flag == 1):
                if init_count == 0:
                     init_count,count_h,bcap_man = auto_init(init_count,gbx, gby, gb_dist, tx, ty, tz, rx, ry, rz, LIDAR_dist1, LIDAR_dist2, debugM)
                print('auto_control')
                auto_control(serial_port,gbx, gby, gb_dist, tx, ty, tz, rx, ry, rz, LIDAR_dist1, LIDAR_dist2, debugM,count_h,bcap_man)
                flag, print_count = keyboard_stop(flag,print_count)
                flag = manual_in_auto(Ctl_com, serial_port, flag)
                bcap_man = manual_ballcapture(bcap_man)

        elif get_key('s'):
            pwm1, pwm2, pwm3, pwm4, dir1, dir2, dir3, dir4 = stop_all()
            serial_port_out(serial_port, pwm1, pwm2, pwm3, pwm4, dir1, dir2, dir3, dir4)
            print("stop all motors")

        elif get_key('m'):
            flag = 2
            while (flag == 2):
                manual_control(Ctl_com,serial_port)
                flag, print_count = keyboard_stop(flag,print_count)

        elif get_key('v'):
            flag = 3
            while (flag == 3):
                variables_change_once()
                flag = 0
                print_count = 1

        elif get_key('t'):
            flag = 4
            while (flag == 4):
                test_function()
                flag, print_count = keyboard_stop(flag, print_count)

        elif get_key('k'):
            break

        if print_count != 0:
            print("No subsystem is running")
            print_count = 0
