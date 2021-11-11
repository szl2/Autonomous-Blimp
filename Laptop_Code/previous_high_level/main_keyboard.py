import time
import serial
import pygame
import simple_pid.PID as PID
import cv2

from constants import *

global ml
ml = 1
if ml == 1:
    import ball_detection.ball_detection as ball_detection

# ========= Serial Port I/O ===========

def serial_port_in(serial_port):
    '''
    Description:
        Take all ESP32_Master serial port's printIn and take all necessary input object

    Input:
        serial_port     :    serial.Serail object

    Output:
        tx, ty, tz, rx, ry, rz, LIDAR_dist, DebugM
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
            LIDAR_dist = val

    line = serial_port.readline()
    debugM = line.decode()

    # DEBUG Verbose
    print("tx:{}".format(tx))
    print("ty:{}".format(ty))
    print("tz:{}".format(tz))
    print("rx:{}".format(rx))
    print("ry:{}".format(ry))
    print("rz:{}".format(rz))
    print("dist:{}".format(LIDAR_dist))
    print(debugM)

    return tx, ty, tz, rx, ry, rz, LIDAR_dist, debugM


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
    if tx == 0 and ty == 0:
        return False
    else:
        return True

def ball_capture(LIDAR_dist):
    '''
    return True if April Tag is detected
    '''
    if (LIDAR_dist < LIDAR_Thres) and (LIDAR_dist > 0):  # Ball captured
        return True
    else:
        return False

def stop_all():
    pwm1, pwm2, pwm3, pwm4 = 0 , 0 , 0 , 0
    dir1, dir2, dir3, dir4 = '+', '-', '+', '-'
    return pwm1, pwm2, pwm3, pwm4, dir1, dir2, dir3, dir4

def move2goal(tx, ty):
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
    inputx = tx / 1.00
    inputy = -1.00 * (ty + AT_goal_Delta) / 1.00 #

    # April Tag Center
    setpoint_x1 = 0.0
    setpoint_y1 = 0.0

    pid_x = PID(kdx_g, kix_g, kpx_g, setpoint = setpoint_x1)
    pid_y = PID(kdx_g, kiy_g, kpy_g, setpoint = setpoint_y1)

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
        dir1 = '-'
        dir2 = '-'
    else:
        dir1 = '+'
        dir2 = '+'

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


def seeking():
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
    pwm1, pwm2, pwm3, pwm4 = 0 , 0 , seeking_speed , seeking_speed
    dir1, dir2, dir3, dir4 = '+', '+', '+', '-'

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
def main_control(gbx, gby, gb_dist, tx, ty, tz, rx, ry, rz, LIDAR_dist, debugM):
    '''
    Description:
        Given green ball information and AT information, the main control logic
        to manuver the blimp motors

    Input:
        gbx, gby, gb_dist                   :   green ball information
        tx, ty, tz, rx, ry, rz, LIDAR_dist  :   AirTag information
        debugM                              :   Debug Message

    Output:
        pwm1, pwm2, pwm3, pwm4, dir1, dir2, dir3, dir4  :   Blimp motor manuver parameters
    '''
    # placeholder
    pwm1, pwm2, pwm3, pwm4 = 0 , 0 , 0 , 0
    dir1, dir2, dir3, dir4 = '+', '-', '+', '-'

    ballDetect  = ball_detect(gbx, gby)
    ballCapture = ball_capture(LIDAR_dist)
    goalDetect  = goal_detect(tx, ty)
    ballCapture = 1
    if ballCapture: # Ball captured
        if goalDetect:  # Goal detected
            # stop_all()  # Debug
            pwm1, pwm2, pwm3, pwm4, dir1, dir2, dir3, dir4 = move2goal(tx, ty)
        else:  # Goal not detected
            # stop_all()  # Debug
            pwm1, pwm2, pwm3, pwm4, dir1, dir2, dir3, dir4 = seeking()
    else:  # Ball not captured
        if ballDetect:  # Ball detected
            pwm1, pwm2, pwm3, pwm4, dir1, dir2, dir3, dir4 = move2ball(gbx,gby,gb_dist)
        else:  # Ball not detected
            # stop_all()  # Debug
            pwm1, pwm2, pwm3, pwm4, dir1, dir2, dir3, dir4 = seeking()

    return pwm1, pwm2, pwm3, pwm4, dir1, dir2, dir3, dir4

# ============= keyboard interruption ===================

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

def auto_control(serial_port,gbx, gby, gb_dist, tx, ty, tz, rx, ry, rz, LIDAR_dist, debugM):
    # ===== STEP 1: TAKE ALL INPUT =====
    # gbx, gby, gb_dist = ball_detection.detectLive(model, minDetectionScore, showSight = True)
    line = serial_port.readline()

    if line == b'SERIAL_IN_START\r\n':
        tx, ty, tz, rx, ry, rz, LIDAR_dist, debugM = serial_port_in(serial_port)
        print("gbx,gby:{},{}".format(gbx, gby))
        time.sleep(waitTime)

    # ===== STEP 2: MAIN CONTROL LOOP =====
    pwm1, pwm2, pwm3, pwm4, dir1, dir2, dir3, dir4 = main_control(gbx, gby, gb_dist, tx, ty, tz, rx, ry, rz, LIDAR_dist,
                                                                  debugM)

    # ===== STEP 3: FEED ALL OUTPUT =====
    serial_port_out(serial_port, pwm1, pwm2, pwm3, pwm4, dir1, dir2, dir3, dir4)

    time.sleep(waitTime)

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

    pwm1, pwm2, pwm3, pwm4, dir1, dir2, dir3, dir4 = decode_ctl(Ctl_com)

    waitTime = 0.05
    # changed
    gbx, gby, gb_dist = ball_detection.detectLive(model, minDetectionScore, showSight = True)

    serial_port_out(serial_port, pwm1, pwm2, pwm3, pwm4, dir1, dir2, dir3, dir4)

    time.sleep(waitTime)


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

def auto_init(init_count,gbx, gby, gb_dist, tx, ty, tz, rx, ry, rz, LIDAR_dist, debugM):
    if ml == 1:
        gbx, gby, gb_dist = ball_detection.detectLive(model, minDetectionScore, showSight = True)
    pwm1, pwm2, pwm3, pwm4, dir1, dir2, dir3, dir4 = main_control(gbx, gby, gb_dist, tx, ty, tz, rx, ry, rz, LIDAR_dist, debugM)
    serial_port_out(serial_port, pwm1, pwm2, pwm3, pwm4, dir1, dir2, dir3, dir4)
    init_count += 1
    return init_count
# ===== Main Function =====
if __name__ == '__main__':
    # =========== SET UP ============
    # Defining Variables for ESP 32 Serial I/O
    PORT = "COM6" # Based on your own serial port number
    serial_port = serial.Serial(PORT, 115200)
    serial_port.close()
    serial_port.open()

    # Weit Time
    waitTime = 0.05

    # Loading the PyTorch ML model for ball detection
    if ml == 1:
        model = ball_detection.returnModel(device, labelSet, modelLoc, modelFile)

    # =========== DECLARE VARIABLES ===========
    # ESP CAM In
    global gbx,gby,gb_dist
    gbx, gby  = -1, -1   # by default (-1 means no found green ball)
    gb_dist = -1         # by default (-1 means no found green ball)

    # Serial Port In
    tx, ty, tz = 100000, 100000, 100000  # by default (0 means no found AirTag)
    rx, ry, rz = 0, 0, 0
    LIDAR_dist = 0
    debugM = 'Testing'

    # Serial Port Out
    pwm1, pwm2, pwm3, pwm4 = 0 , 0 , 0 , 0   # Not moving
    dir1, dir2, dir3, dir4 = '+', '+', '+', '+'

    # Trigger the ESP32_SLAVE to talk first


    Ctl_com = [0, 0, 0, 0, "+", "+", "+", "+"]
    flag = 0
    print_count = 1
    init_count = 0
    global start_speed
    start_speed = 70

    init()
    # =========== LOOP FOREVER===========
    while True:

        if get_key('a'):
            flag = 1
            while (flag == 1):
                if init_count == 0:
                     init_count = auto_init(init_count,gbx, gby, gb_dist, tx, ty, tz, rx, ry, rz, LIDAR_dist, debugM)

                auto_control(serial_port,gbx, gby, gb_dist, tx, ty, tz, rx, ry, rz, LIDAR_dist, debugM)
                flag, print_count = keyboard_stop(flag,print_count)

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

        elif get_key('k'):
            break

        if print_count is not 0:
            print("No subsystem is running")
            print_count = 0
