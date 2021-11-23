import os
import time
import blimp
import pygame
import serial
import numpy as np
import pid.PID as PID
import ball_detection.ball_detection as ball_detection
from constants import *
from data_logging import *
from datetime import datetime
from apriltag_detection import get_AT_6DOF_info
# from imageTread_AT_multiple import get_AT_6DOF_info_list

logTime = datetime.now().strftime('%Y%m%d-%H%M%S')
logPath = './logs/' + logTime + '/'
if not os.path.exists('./logs/'):
    os.makedirs('./logs/')
if not os.path.exists(logPath):
    os.makedirs(logPath)
logger, formatter = logger_setup()
logger, fileHandler = fileHandler_setup(logger, formatter, logPath + 'logs-' + logTime + '.log')

##DEFINE THE GLOBAL VARIABLES AND OBJECTS
url_gb = 'http://10.0.0.10/cam-hi.jpg'
url_AT = 'http://10.0.0.29/cam-hi.jpg'

myBlimp = blimp.Blimp(logger)

global ballCapture
ballCapture = 0

global seekingTimeTracker
seekingTimeTracker = time.time()

global seekingTimerStatus
seekingTimerStatus = False

global GLOBALSTATE
GLOBALSTATE = "REGULARBALLSEEKING"

global timeTracker
timeTracker = 0

global lastknown_gbx
lastknown_gbx = 0

global lastknown_gby
lastknown_gby = 0

global lastknown_tx
lastknown_tx = 0

global lastknown_ty
lastknown_ty = 0

global lastknown_tz
lastknown_tz = 0 #These 3 are updated in the move2goal function

dateTimeObj = datetime.now()

def manual_vertical():
    pwm1 = convertAxis(axes_win.get("vertical"))
    pwm2 = pwm1
    if pwm1 > 0:
        dir1 = "+"
        dir2 = "+"
    else:
        dir1 = "-"
        dir2 = "-"
    return abs(pwm1), abs(pwm2), dir1, dir2

def manual_horizontal():
    data_f = convertAxis(axes_win.get("forward"))
    data_t = convertAxis(axes_win.get("turn"))
    if abs(data_f) > abs(data_t):
        pwm3 = data_f
        pwm4 = pwm3
        if pwm3 > 0:
            dir3 = "+"
            dir4 = "+"
        else:
            dir3 = "-"
            dir4 = "-"
    elif abs(data_f) < abs(data_t):
        pwm3 = data_t
        pwm4 = pwm3
        if pwm3 > 0:
            dir3 = "-"
            dir4 = "+"
        else:
            dir3 = "+"
            dir4 = "-"
    else:
        pwm3 = 0
        pwm4 = 0
        dir3 = "+"
        dir4 = "+"
    return abs(pwm3),abs(pwm4),dir3,dir4

def manual_control():
    # gbx, gby, gb_dist, frame = ball_detection.detectLive(url_gb, model, minDetectionScore, showSight=True, detectFlag=False)
    # log_image(logPath, 'ball_detection', datetime.now().strftime('%Y%m%d-%H%M%S'), frame)
    # log_variable(logger, '[gbx, gby, gb_dist]', [gbx, gby, gb_dist])

    # tid, tx, ty, tz, rx, ry, rz, frame = get_AT_6DOF_info(url_AT, logger, detectFlag=False)
    # log_image(logPath, 'apriltag_detection', datetime.now().strftime('%Y%m%d-%H%M%S'), frame)
    # log_variable(logger, '[tid, tx, ty, tz, rx, ry, rz]', [tid, tx, ty, tz, rx, ry, rz])

    pwm1,pwm2,dir1,dir2 = manual_vertical()
    pwm3,pwm4,dir3,dir4 = manual_horizontal()
    return pwm1, pwm2, pwm3, pwm4, dir1, dir2, dir3, dir4

def nonlinear_mapping(val):
    abs_val = abs(val)
    output = 1 - abs_val ** 0.5
    if val > 0:
        return val
    else:
        return -1*val

def rescale(oldValue, newMin=0, newMax=255):
    oldMax = 1
    oldMin = -1

    oldRange = (oldMax - oldMin)
    newRange = (newMax - newMin)
    newValue = (((oldValue - oldMin) * newRange) / oldRange) + newMin

    newValue = nonlinear_mapping(newValue /(newMax+0.0)) * newValue

    return int(newValue)

def convertThrust(data):
    return rescale(data, -255, 255)

def convertAxis(axes):
    data = joystick.get_axis(axes)*-1
    pwm = convertThrust(data)
    return pwm

def go_back2_upper_level(flag_b,print_count_b):
    if joystick.get_button(axes_win.get('button_back')):
        log_variable(logger, 'OFF MODE')
        flag_b = 0
        print_count_b = 1
    return flag_b,print_count_b

def pygame_init(done):
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            done = True
    return done

def stop_all():
    pwm1, pwm2, pwm3, pwm4 = 0 , 0 , 0 , 0
    dir1, dir2, dir3, dir4 = '+', '-', '+', '-'
    return pwm1, pwm2, pwm3, pwm4, dir1, dir2, dir3, dir4

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

def serial_port_out(Ctl_com):
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
    log_variable(logger, 'Ctl_com', Ctl_com)
    print(dateTimeObj, end='')
    pwm1, pwm2, pwm3, pwm4, dir1, dir2, dir3, dir4 = decode_ctl(Ctl_com)
    myBlimp.set_BlimpMotors(pwm1, pwm2, pwm3, pwm4, dir1, dir2, dir3, dir4)

def serial_port_in():
    '''
    Description:
        Take all ESP32_Master serial port's printIn and take all necessary input object

    Input:
        serial_port     :    serial.Serail object

    Output:
        tx, ty, tz, rx, ry, rz, LIDAR_dist1, LIDAR_dist2, DebugM
    '''

    # Serial Port In
    LIDAR_dist1 = myBlimp.get_LidarDist1();
    LIDAR_dist2 = 0
    log_variable(logger, 'LIDAR_dist1', LIDAR_dist1)
    log_variable(logger, 'LIDAR_dist2', LIDAR_dist2)
    # DEBUG Verbose
    print(dateTimeObj, "Input: LIDAR_dist1:", LIDAR_dist1,  LIDAR_dist2)
    return LIDAR_dist1, LIDAR_dist2

def test_function():
    # gbx, gby, gb_dist, frame = ball_detection.detectLive(url_gb, model, minDetectionScore, showSight=True)
    # log_image(logPath, 'ball_detection', datetime.now().strftime('%Y%m%d-%H%M%S'), frame)
    # log_variable(logger, '[gbx, gby, gb_dist]', [gbx, gby, gb_dist])

    tid, tx, ty, tz, rx, ry, rz, frame = get_AT_6DOF_info(url_AT, logger)
    log_image(logPath, 'apriltag_detection', datetime.now().strftime('%Y%m%d-%H%M%S'), frame)
    log_variable(logger, '[tid, tx, ty, tz, rx, ry, rz]', [tid, tx, ty, tz, rx, ry, rz])

def auto_init( auto_init_count):
    print("This is auto_init")
    auto_init_count += 1
    bcap_man = -1 # default: use lidar to determine
    # Ctl_com = main_control(gbx, gby, gb_dist, tx, ty, tz, rx, ry, rz, LIDAR_dist1, LIDAR_dist2, debugM, bcap_man)
    # serial_port_out(Ctl_com)
    return auto_init_count,bcap_man

def manual_in_auto(Ctl_com, flag):
    done = 1
    if joystick.get_button(axes_win.get('button_LB')):
        log_variable(logger, 'MANUAL IN AUTO MODE')
        print("manual in auto")
        flag = 12
    while (flag==12):
        done = pygame_init(done)
        Ctl_com = manual_control()
        serial_port_out(Ctl_com)
        flag = manual_return2auto(flag)

    # gbx, gby, gb_dist, frame = ball_detection.detectLive(url_gb, model, minDetectionScore, showSight=True, detectFlag=False)
    # log_image(logPath, 'ball_detection', datetime.now().strftime('%Y%m%d-%H%M%S'), frame)
    # log_variable(logger, '[gbx, gby, gb_dist]', [gbx, gby, gb_dist])

    # tid, tx, ty, tz, rx, ry, rz, frame = get_AT_6DOF_info(url_AT, logger, detectFlag=False)
    # log_image(logPath, 'apriltag_detection', datetime.now().strftime('%Y%m%d-%H%M%S'), frame)
    # log_variable(logger, '[tid, tx, ty, tz, rx, ry, rz]', [tid, tx, ty, tz, rx, ry, rz])
    return flag

def manual_return2auto(flag):
    if joystick.get_button(axes_win.get('button_a')):
        log_variable(logger, 'AUTO MODE: From MANUAL IN AUTO')
        flag = 1
    return flag

def manual_ballcapture(bcap_man):
    if joystick.get_button(axes_win.get('button_b')): # the green ball is capture
        bcap_man = 1
    elif joystick.get_button(axes_win.get('button_x')):
        bcap_man = 0
    elif joystick.get_button(axes_win.get('button_y')):
        bcap_man = -1
    log_variable(logger, 'bcap_man', bcap_man)
    return bcap_man

# ====== Logic-directing Functions ====
def ball_detect(gbx, gby):
    '''
    return True if green ball is detected
    '''
    if gbx == -1 and gby == -1:
        return False
    else:
        return True

def goal_detect(tx, ty, tid):
    '''
    return True if April Tag is detected
    '''
    if tx == 100000 and ty == 100000:
        return False
    else:
        # if tid == 1 or tid == 2 or tid == 3:
        #     return True
        # else:
        #     return False
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

def emgency_movement():
    #press the trigger it will go down or up for 3 seconds
    # temp_time = 0
    # how_long_blimp_move_vertically = 3 #in seconds
    speed = 200
    if joystick.get_axis(axes_win.get('trigger_LT')) > 0.5:
        #GO DOWN  FOR 3 seconds
        Ctl_com = [speed, speed, 0, 0, "-", "-", "+", "+"]
        serial_port_out(Ctl_com)
        time.sleep(3)

    elif joystick.get_axis(axes_win.get('trigger_RT')) > 0.5:
        #GO UP FOR 3 SECONDS
        Ctl_com = [speed, speed, 0, 0, "+", "+", "+", "+"]
        serial_port_out(Ctl_com)
        time.sleep(3)

def auto_control( gbx, gby, gb_dist, tx, ty, tz, rx, ry, rz, LIDAR_dist1, LIDAR_dist2, debugM):
    global ballCapture

    tid = -1

    if ballCapture == 0:
        print("gb screen")
        gbx, gby, gb_dist, frame = ball_detection.detectLive(url_gb, model, minDetectionScore, showSight=True)
        log_image(logPath, 'ball_detection', datetime.now().strftime('%Y%m%d-%H%M%S'), frame)
        log_variable(logger, '[gbx, gby, gb_dist]', [gbx, gby, gb_dist])
        print("gbx,gby:{},{}".format(gbx, gby))
    else:
        print("AT screen")
        LIDAR_dist1, LIDAR_dist2 = serial_port_in()
        tid, tx, ty, tz, rx, ry, rz, frame = get_AT_6DOF_info(url_AT, logger)
        log_image(logPath, 'apriltag_detection', datetime.now().strftime('%Y%m%d-%H%M%S'), frame)
        log_variable(logger, '[tid, tx, ty, tz, rx, ry, rz]', [tid, tx, ty, tz, rx, ry, rz])

    # main auto control loop
    Ctl_com = main_control(gbx, gby, gb_dist, tx, ty, tz, rx, ry, rz, LIDAR_dist1, LIDAR_dist2, debugM, bcap_man, tid)

    return Ctl_com

def determineBallCaptureStatus(bcap_man, LIDAR_dist1):
    bc = 10 #random number
    if bcap_man == 1:
        bc = 1  # Manually determine Ball captured
    elif bcap_man == 0:
        bc = 0 # Ball not captured
    elif bcap_man == -1:
        bc = ball_capture(LIDAR_dist1)
    return bc

def main_control(gbx, gby, gb_dist, tx, ty, tz, rx, ry, rz, LIDAR_dist1, LIDAR_dist2, debugM, bcap_man, tid):
    #define the global variables used
    global seekingTimeTracker
    global GLOBALSTATE
    global ballCapture
    global timeTracker
    global lastknown_tx
    global lastknown_ty
    global lastknown_tz
    global lastknown_gbx
    global lastknown_gby

    #print('in main_control')
    pwm1, pwm2, pwm3, pwm4 = 0 , 0 , 0 , 0
    dir1, dir2, dir3, dir4 = '+', '-', '+', '-'

    # detection flag: ballDetect,goalDetect,ballCapture
    ballDetect = ball_detect(gbx, gby)
    goalDetect = goal_detect(tx, ty, tid)
    ballCapture = determineBallCaptureStatus(bcap_man, LIDAR_dist1)

    print("THE CURRENT STATE IS: ", GLOBALSTATE)
    if ballCapture: # Ball captured
        if (GLOBALSTATE == "SPIRALDOWNBALLSEEKING" or GLOBALSTATE == "SPIRALUPBALLSEEKING" or GLOBALSTATE == "MOVINGTOBALL" or GLOBALSTATE == "CLOSETOBALLSEEKING"):
            GLOBALSTATE = "SPIRALUPGOALSEEKING"
        if goalDetect:  # Goal detected
            GLOBALSTATE = "MOVINGTOGOAL"
            timeTracker = time.time()
            pwm1, pwm2, pwm3, pwm4, dir1, dir2, dir3, dir4 = move2goal(tx, ty,tz) #fix
        else:  # Goal not detected
            #if the goal was within hovering distance + 1 and last state was move2goal
            if (GLOBALSTATE == "MOVINGTOGOAL" or GLOBALSTATE == "FARFROMGOALSEEKING"):
                if (lastknown_tz < goalHoverDistance): #close enough
                    GLOBALSTATE = "CLOSETOGOALSEEKING"
                else: #we are far from the goal
                    currentTime = time.time()
                    GLOBALSTATE = "FARFROMGOALSEEKING"
                    if (currentTime - timeTracker < TrytoFindGoalAgainTime):
                        pwm1, pwm2, pwm3, pwm4, dir1, dir2, dir3, dir4 = goalLostTrytoFindAgain()
                    else:
                        GLOBALSTATE = "CLOSETOGOALSEEKING"
            elif (GLOBALSTATE == "CLOSETOGOALSEEKING"):
                currentTime = time.time()
                if (currentTime - timeTracker < CloseGoalSeekingTime):
                    pwm1, pwm2, pwm3, pwm4, dir1, dir2, dir3, dir4 = goalLostTrytoFindAgain()
                else: #done with trying close seeking
                    timeTracker = time.time()
                    if (lastknown_ty > 0):
                        GLOBALSTATE = "SPIRALDOWNGOALSEEKING"
                    else:
                        GLOBALSTATE = "SPIRALUPGOALSEEKING"
            elif(GLOBALSTATE == "SPIRALUPGOALSEEKING"): #not in hovering distance and was NOT spiraling up for some time
                currentTime = time.time()
                if (currentTime - timeTracker < SpiralUpSeekingTime):
                    pwm1, pwm2, pwm3, pwm4, dir1, dir2, dir3, dir4 = rotate_up()
                else:
                    timeTracker = time.time()
                    GLOBALSTATE = "SPIRALDOWNGOALSEEKING"
            else: #not in hovering distance and was NOT spiraling up for some time
                currentTime = time.time()
                if (currentTime - timeTracker < SpiralDownSeekingTime):
                    pwm1, pwm2, pwm3, pwm4, dir1, dir2, dir3, dir4 = rotate_down()
                else:
                    timeTracker = time.time()
                    GLOBALSTATE = "SPIRALUPGOALSEEKING"
    else:  # Ball not captured
        if (GLOBALSTATE == "MOVINGTOGOAL" or GLOBALSTATE == "CLOSETOGOALSEEKING" or GLOBALSTATE == "SPIRALUPGOALSEEKING" or GLOBALSTATE == "SPIRALDOWNGOALSEEKING"):
            GLOBALSTATE = "SPIRALUPBALLSEEKING"
        if ballDetect:  # Ball detected
            timeTracker = time.time()
            GLOBALSTATE = "MOVINGTOBALL"
            pwm1, pwm2, pwm3, pwm4, dir1, dir2, dir3, dir4 = move2ball(gbx,gby,gb_dist)
        else:  # Ball not detected
            if (GLOBALSTATE == "CLOSETOBALLSEEKING" or GLOBALSTATE == "MOVINGTOBALL"):
                GLOBALSTATE = "CLOSETOBALLSEEKING"
                currentTime = time.time()
                #print(timeTracker - currentTime)
                if (currentTime - timeTracker < CloseBallSeekingTime/2.0):
                    if (goFortheBall == False):
                        pwm1, pwm2, pwm3, pwm4, dir1, dir2, dir3, dir4 = close2ballSeeking()
                    else:
                        pwm1, pwm2, pwm3, pwm4, dir1, dir2, dir3, dir4 = reverse()
                elif(currentTime - timeTracker < CloseBallSeekingTime):
                    pwm1, pwm2, pwm3, pwm4, dir1, dir2, dir3, dir4 = close2ballSeeking()
                #     pwm1, pwm2, pwm3, pwm4, dir1, dir2, dir3, dir4 = close2ballSeeking()
                else:
                    timeTracker = time.time()
                    GLOBALSTATE = "SPIRALUPBALLSEEKING"
            elif(GLOBALSTATE == "SPIRALUPBALLSEEKING"): #not in hovering distance and was NOT spiraling up for some time
                currentTime = time.time()
                if (currentTime - timeTracker < SpiralUpSeekingTime):
                    pwm1, pwm2, pwm3, pwm4, dir1, dir2, dir3, dir4 = rotate_up()
                else:
                    timeTracker = time.time()
                    GLOBALSTATE = "SPIRALDOWNBALLSEEKING"
            else: #not in hovering distance and was NOT spiraling up for some time
                currentTime = time.time()
                if (currentTime - timeTracker < SpiralDownSeekingTime):
                    pwm1, pwm2, pwm3, pwm4, dir1, dir2, dir3, dir4 = rotate_down()
                else:
                    timeTracker = time.time()
                    GLOBALSTATE = "SPIRALUPBALLSEEKING"
    log_variable(logger, 'GLOBALSTATE: ', GLOBALSTATE)
    return pwm1, pwm2, pwm3, pwm4, dir1, dir2, dir3, dir4

# ====== action Functions ====
def reverse():
    pwm1, pwm2, pwm3, pwm4 = 100, 100, CloseBallReverseSpeed, CloseBallReverseSpeed
    dir1, dir2, dir3, dir4 = '+', '+', '-', '-'
    return pwm1, pwm2, pwm3, pwm4, dir1, dir2, dir3, dir4

def scaling(val, initstart, initend, finalstart, finalend):
    result = (finalend - finalstart)*val/(initend - initstart)
    return result

def goalLostTrytoFindAgain():
    global lastknown_tx
    global lastknown_ty
    pwm1, pwm2, pwm3, pwm4 = 0, 0, 0, 0
    dir1, dir2, dir3, dir4 = '+', '+', '+', '-'

    x_error = lastknown_tx
    y_error = lastknown_ty

    # x_input = scaling(abs(x_error), 0, 5, 0, MAX_SPEED)
    # y_input = scaling(abs(y_error), 0, 5, 0, MAX_SPEED)
    #
    # pwm1 = min(255, y_input)
    # pwm2 = min(255, y_input)
    # pwm3 = min(255, x_input)
    # pwm4 = min(255, x_input)
    pwm1, pwm2 = 255, 255
    pwm3, pwm4 = 255, 255

    if y_error > 0: #blimp is below
        dir1 = '-'
        dir2 = '-'
    else: #blimp is above
        dir1 = '+'
        dir2 = '+'

    if x_error > 0: #need to move right
        dir3 = '+'
        dir4 = '-'
    else:
        dir3 = '-'
        dir4 = '+'
    return pwm1, pwm2, pwm3, pwm4, dir1, dir2, dir3, dir4

def close2ballSeeking(): #ORIENTATION RIGHT
    global lastknown_gbx
    global lastknown_gby
    pwm1, pwm2, pwm3, pwm4 = 0, 0, 0, 0
    dir1, dir2, dir3, dir4 = '+', '+', '+', '-'

    x_error = lastknown_gbx - (RESOLUTION_W/2.0)
    y_error = lastknown_gby - (RESOLUTION_H/2.0)

    x_input = scaling(abs(x_error), 0, RESOLUTION_W/2.0, 0, MAX_SPEED)
    y_input = scaling(abs(y_error), 0, RESOLUTION_H/2.0, 0, MAX_SPEED)

    pwm1 = y_input
    pwm2 = y_input
    pwm3 = x_input
    pwm4 = x_input

    if y_error > 0: #blimp is above
        dir1 = '-'
        dir2 = '-'
    else: #blimp is below
        dir1 = '+'
        dir2 = '+'

    if x_error > 0: #need to move right
        dir3 = '+'
        dir4 = '-'
    else:
        dir3 = '-'
        dir4 = '+'
    return pwm1, pwm2, pwm3, pwm4, dir1, dir2, dir3, dir4

def move2goal(tx, ty,tz):

    global lastknown_tx
    global lastknown_ty
    global lastknown_tz
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
    lastknown_tx = tx
    lastknown_ty = ty
    lastknown_tz = tz
    # April Tag Center
    setpoint_x1 = 0.0
    setpoint_y1 = 0.0 + AT_goal_Delta

    inputx = tx * 100/ 1.00
    inputy =  (ty * 100) / 1.00 #

    if (lookAheadPID_Goal == True):
            inputx = (inputx+setpoint_x1)/2.0
            inputy = (inputx+setpoint_y1)/2.0

    pid_x = PID(kpx_g, kix_g, kdx_g, setpoint = setpoint_x1)
    pid_y = PID(kpy_g, kiy_g, kdy_g, setpoint = setpoint_y1)

    pid_x.auto_mode = True
    pid_y.auto_mode = True
    #pid_x.set_auto_mode(True, last_output = 8.0)
    pid_x.output_limits = (-255,255)
    pid_y.output_limits = (-255,255)

    outputx = pid_x(inputx)
    outputy = pid_y(inputy)

    log_variable(logger, '[inputx, inputy]', [inputx, inputy])
    log_variable(logger, '[outputx, outputy]', [outputx, outputy])

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

    val = scaling(tz*100, 0, goalHoverDistance*100, 0, 255)
    val = 255 - val
    Hlspeed = -1 * outputx - val
    Hrspeed =  1 * outputx - val
    if (Hlspeed > 0):
        Hdir3 = '+'
    else:
        Hdir3 = '-'
    if (rspeed > 0):
        Hdir4 = '+'
    else:
        Hdir4 = '-'

    if (goFortheGoal == True):
        return int(pwm1), int(pwm2), int(pwm3), int(pwm4), dir1, dir2, dir3, dir4
    else:
        if (tz > goalHoverDistance):
            return int(pwm1), int(pwm2), int(pwm3), int(pwm4), dir1, dir2, dir3, dir4
        else:
            return int(pwm1), int(pwm2), int(abs(Hlspeed)), int(abs(Hrspeed)), dir1, dir2, Hdir3, Hdir4

def rotate_one_direction():
    pwm1, pwm2, pwm3, pwm4 = 0, 0, seeking_speed, seeking_speed
    dir1, dir2, dir3, dir4 = '+', '+', '+', '-'
    return int(pwm1), int(pwm2), int(pwm3), int(pwm4), dir1, dir2, dir3, dir4

def rotate_up():
    pwm1, pwm2, pwm3, pwm4 = Rotate_VerticalSpeed, Rotate_VerticalSpeed, Rotate_SpinningSpeed, Rotate_SpinningSpeed
    dir1, dir2, dir3, dir4 = '+', '+', '+', '-'
    return int(pwm1), int(pwm2), int(pwm3), int(pwm4), dir1, dir2, dir3, dir4

def rotate_down():
    pwm1, pwm2, pwm3, pwm4 = Rotate_VerticalSpeed, Rotate_VerticalSpeed, Rotate_SpinningSpeed, Rotate_SpinningSpeed
    dir1, dir2, dir3, dir4 = '-', '-', '+', '-'
    return int(pwm1), int(pwm2), int(pwm3), int(pwm4), dir1, dir2, dir3, dir4

def move2ball(gbx, gby, gb_dist):
    global lastknown_gbx
    global lastknown_gby
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

    lastknown_gbx = gbx
    lastknown_gby = gby

    # ESP-Cam Center
    setpoint_x = 400
    setpoint_y = 300

    if (lookAheadPID_Ball == True):
        inputx = (inputx+setpoint_x)/2.0
        inputy = (inputx+setpoint_y)/2.0

    pid_x = PID(kpx, kix, kdx, setpoint = setpoint_x)
    pid_y = PID(kpy, kiy, kdy, setpoint = setpoint_y)

    pid_x.auto_mode = True
    pid_y.auto_mode = True
    #pid_x.set_auto_mode(True, last_output = 8.0)
    pid_x.output_limits = (-255,255)
    pid_y.output_limits = (-255,255)

    outputx = pid_x(inputx)
    outputy = pid_y(inputy)

    log_variable(logger, '[inputx, inputy]', [inputx, inputy])
    log_variable(logger, '[outputx, outputy]', [outputx, outputy])

    # Vertical
    pwm1 = abs(outputy)
    pwm2 = abs(outputy)

    if (outputy > 0):
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

    val = scaling(gb_dist, 0, 400, 0, 255)
    val = 255 - val
    Hlspeed = -1 * outputx - val
    Hrspeed =  1 * outputx - val
    if (Hlspeed > 0):
        Hdir3 = '+'
    else:
        Hdir3 = '-'
    if (rspeed > 0):
        Hdir4 = '+'
    else:
        Hdir4 = '-'

    #print("this is the gb dist: ", gb_dist)
    if goFortheBall == True:
        return int(pwm1), int(pwm2), int(pwm3), int(pwm4), dir1, dir2, dir3, dir4
    else: #need to create algorithm for ball hovering
        if (gb_dist > ballHoverDistance):
            return int(pwm1), int(pwm2), int(pwm3), int(pwm4), dir1, dir2, dir3, dir4
        else:
            #print("HOVERING")
            return int(pwm1), int(pwm2), int(abs(Hlspeed)), int(abs(Hrspeed)), dir1, dir2, Hdir3, Hdir4

if __name__ == '__main__':
    # =========== SET UP ============
    waitTime = 0.01

    # define game controller axes value
    axes_win = dict(
        forward = 1,
        turn = 0,
        vertical = 3,
        button_a = 0, # auto control
        button_b = 1, # manual_ball_capture
        button_x = 2, # manual_no_ball_capture
        button_y = 3, # let the lidar to determine ball caputure
        button_LB = 4, # manual control
        button_RB = 5, # stop all motors
        button_back = 6, # go back to the first level
        button_start = 7, # test camera core function
        trigger_LT = 4, # prevent blimp from going too high
        trigger_RT = 5, # prevent blimp from going too low
    )

    # Set up joystick
    pygame.init()
    pygame.joystick.init()
    done = False

    # Loading the PyTorch ML model for ball detection
    ball_detection.modifyCore()
    model = ball_detection.returnModel(device, labelSet, modelLoc, modelFile)

    # =========== DECLARE VARIABLES ===========
    # game controller id
    controller_id = 0 # default is 0 if you plug in only one game controller

    # ESP CAM In
    gbx, gby = -1, -1  # by default (-1 means no found green ball)
    gb_dist = -1  # by default (-1 means no found green ball)

    # Serial Port In
    tx, ty, tz = 100000, 100000, 100000  # by default (0 means no found AirTag)
    rx, ry, rz = 0, 0, 0
    LIDAR_dist1 = 0
    LIDAR_dist2 = 0
    debugM = 'Testing'

    # control commands
    Ctl_com = [0, 0, 0, 0, "+", "+", "+", "+"]

    # flag variables
    flag = 0
    print_count = 0
    auto_init_count = 0
    bcap_man = -1

    while not done:
        done = pygame_init(done)

        joystick = pygame.joystick.Joystick(controller_id)
        joystick.init()

        if joystick.get_button(axes_win.get('button_LB')) and joystick.get_button(axes_win.get('button_RB')):
            log_variable(logger, 'PROGRAM KILL')
            print("kill the program")
            done = True

        elif joystick.get_button(axes_win.get('button_a')):
            log_variable(logger, 'AUTONOMOUS MODE')
            print("auto_control")
            flag = 1
            while (flag==1):
                done = pygame_init(done)
                Ctl_com = auto_control( gbx, gby, gb_dist, tx, ty, tz, rx, ry, rz, LIDAR_dist1, LIDAR_dist2, debugM)
                serial_port_out(Ctl_com) # send the control data out
                emgency_movement()
                flag, print_count = go_back2_upper_level(flag, print_count) # go back to upper level by pressing "back"
                flag = manual_in_auto(Ctl_com,   flag) # turn into manual control in auto_control
                bcap_man = manual_ballcapture(bcap_man)    # manually determine ball capture status

        elif joystick.get_button(axes_win.get('button_LB')):
            log_variable(logger, 'MANUAL MODE')
            print("manual control")
            flag = 2
            while (flag == 2):
                done = pygame_init(done)
                Ctl_com = manual_control()
                serial_port_out(Ctl_com)
                flag, print_count = go_back2_upper_level(flag, print_count)

        elif joystick.get_button(axes_win.get('button_start')):
            log_variable(logger, 'CAMERA TESTING MODE')
            print("test the camera core function")
            flag = 3
            while (flag == 3):
                done = pygame_init(done)
                test_function()
                flag, print_count = go_back2_upper_level(flag, print_count)

        elif joystick.get_button(axes_win.get('button_RB')):
            log_variable(logger, 'STOP ALL MOTORS')
            print("stop all motors")
            Ctl_com = stop_all()
            serial_port_out(Ctl_com)

        pygame.time.wait(50)

        if flag == 0:
            log_variable(logger, 'OFF MODE')
        if print_count != 0:
            print("No subsystem is running")
            print_count = 0
        pygame.time.wait(50)
        log_variable(logger)

    pygame.quit()
