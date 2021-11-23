import pygame
import serial
import time
import simple_pid.PID as PID
from constants import *
from ESP32_AT.imageTread_AT import get_AT_6DOF_info
from ESP32_AT.imageTread_AT_multiple import get_AT_6DOF_info_list

global mc_print,ml_on,esp_cam_on,feather_data_on

mc_print = 1 # manual control print flag
ml_on = 1
feather_data_on = 1

if ml_on == 1:
    import ball_detection.ball_detection as ball_detection

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

def manual_control(serial_port):
    pwm1,pwm2,dir1,dir2 = manual_vertical()
    pwm3,pwm4,dir3,dir4 = manual_horizontal()
    if mc_print == 1:
        print("_______________________________________________")
        print("pwm1,pwm2,pwm3,pwm4:{},{},{},{}".format(pwm1,pwm2,pwm3,pwm4))
        print("dir1,dir2,dir3,dir4:{},{},{},{}".format(dir1,dir2,dir3,dir4))
        print("_______________________________________________")
    return pwm1, pwm2, pwm3, pwm4, dir1, dir2, dir3, dir4

def rescale(oldValue, newMin=0, newMax=255):
    oldMax = 1
    oldMin = -1

    oldRange = (oldMax - oldMin)
    newRange = (newMax - newMin)
    newValue = (((oldValue - oldMin) * newRange) / oldRange) + newMin
    return int(newValue)

def convertThrust(data):
    return rescale(data, -255, 255)

def convertAxis(axes):
    data = joystick.get_axis(axes)*-1
    pwm = convertThrust(data)
    return pwm

def go_back2_upper_level(flag_b,print_count_b):
    if joystick.get_button(axes_win.get('button_back')):
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

def serial_port_out(serial_port, Ctl_com):
    pwm1, pwm2, pwm3, pwm4, dir1, dir2, dir3, dir4 = decode_ctl(Ctl_com)
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

def test_function():

    # tid, tx, ty, tz, rx, ry, rz = get_AT_6DOF_info(url_AT)
    tids, txs, tys, tzs, rxs, rys, rzs = get_AT_6DOF_info_list(url_AT)
    gbx, gby, gb_dist = ball_detection.detectLive(url_gb, model, minDetectionScore, showSight=True)

    print("testing new function")
    print("-----------------------")
    # print("tid:{}".format(tid))
    # print("tx,ty,tz:{},{},{}".format(tx,ty,tz))
    # print("rx,ry,rz:{},{},{}".format(rx,ry,rz))
    print("tid:{}".format(tids))
    print("tx,ty,tz:{},{},{}".format(txs,tys,tzs))
    print("rx,ry,rz:{},{},{}".format(rxs,rys,rzs))
    print("gbx,gby,gb_dist:{},{},{}".format(gbx,gby,gb_dist))

def auto_init(serial_port,auto_init_count,gbx, gby, gb_dist, tx, ty, tz, rx, ry, rz, LIDAR_dist1, LIDAR_dist2, debugM):
    print("This is auto_init")
    auto_init_count += 1
    bcap_man = -1 # default: use lidar to determine
    Ctl_com = main_control(gbx, gby, gb_dist, tx, ty, tz, rx, ry, rz, LIDAR_dist1, LIDAR_dist2, debugM, bcap_man)
    serial_port_out(serial_port, Ctl_com)
    return auto_init_count,bcap_man

def auto_control(serial_port):
    # Ctl_com = [0, 0, 0, 0, "+", "+", "+", "+"]

    if ml_on == 1:
        gbx, gby, gb_dist = ball_detection.detectLive(url_gb, model, minDetectionScore, showSight=True)
        print("gbx,gby:{},{}".format(gbx, gby))

    line = serial_port.readline()
    if feather_data_on == 1:
        if line == b'SERIAL_IN_START\r\n':
            tx, ty, tz, rx, ry, rz, LIDAR_dist1, LIDAR_dist2, debugM = serial_port_in_v1(serial_port)
            # only getting the Lidar data back
            time.sleep(waitTime) # in second

    if esp_cam_on == 1:
        # ids,txs,tys,tzs,rxs,rys,rzs = get_AT_6DOF_info_list(url_AT)
        tid, tx, ty, tz, rx, ry, rz = get_AT_6DOF_info(url_AT)

    # main auto control loop
    Ctl_com = main_control(gbx, gby, gb_dist, tx, ty, tz, rx, ry, rz, LIDAR_dist1, LIDAR_dist2, debugM, bcap_man)

    return Ctl_com

def manual_in_auto(Ctl_com, serial_port, flag):
    if joystick.get_button(axes_win.get('button_LB')):
        flag = 12
        print("manual in auto")
    while (flag==12):
        done = pygame_init(done)
        Ctl_com = manual_control()
        serial_port_out(serial_port, Ctl_com)
        flag = manual_return2auto(flag)
    return flag

def manual_return2auto(flag):
    if joystick.get_button(axes_win.get('button_a')):
        flag = 1
    return flag

def manual_ballcapture(bcap_man):
    if joystick.get_button(axes_win.get('button_b')): # the green ball is capture
        bcap_man = 1
    elif joystick.get_button(axes_win.get('button_x')):
        bcap_man = 0
    elif joystick.get_button(axes_win.get('button_y')):
        bcap_man = -1
    return bcap_man

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

    for i in range(8):
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
        elif i == 7:
            LIDAR_dist2 = val

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

def main_control(gbx, gby, gb_dist, tx, ty, tz, rx, ry, rz, LIDAR_dist1, LIDAR_dist2, debugM, bcap_man):
    print('in main_control')
    pwm1, pwm2, pwm3, pwm4 = 0 , 0 , 0 , 0
    dir1, dir2, dir3, dir4 = '+', '-', '+', '-'

    # detection flag: ballDetect,goalDetect,ballCapture
    ballDetect = ball_detect(gbx, gby)
    goalDetect = goal_detect(tx, ty)
    if bcap_man == 1:
        ballCapture = 1  # Manually determine Ball captured
    elif bcap_man == 0:
        ballCapture = 0 # Ball not captured
    elif bcap_man == -1:
        ballCapture = ball_capture(LIDAR_dist1)

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

# ====== action Functions ====
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

def rotate_one_direction_ssp(ssp):
    pwm1, pwm2, pwm3, pwm4 = 0, 0, ssp, ssp
    dir1, dir2, dir3, dir4 = '+', '+', '+', '-'

    return int(pwm1), int(pwm2), int(pwm3), int(pwm4), dir1, dir2, dir3, dir4

def rotate_one_direction():
    pwm1, pwm2, pwm3, pwm4 = 0, 0, seeking_speed, seeking_speed
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

if __name__ == '__main__':

    # =========== SET UP ============
    # Defining Variables for ESP 32 Serial I/O
    PORT = "COM9" # for Alienware
    serial_port = serial.Serial(PORT, 115200)
    serial_port.close()
    serial_port.open()

    # Weit Time
    waitTime = 0.05

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
    )
    # print(axes_win.get('forward'))

    # Set up joystick
    pygame.init()
    pygame.joystick.init()
    done = False

    # Loading the PyTorch ML model for ball detection
    if ml_on == 1:
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
    while not done:

        done = pygame_init(done)

        joystick = pygame.joystick.Joystick(controller_id)
        joystick.init()

        if joystick.get_button(axes_win.get('button_start')) and joystick.get_button(axes_win.get('button_back')):
            print("kill the program")
            done = True

        elif joystick.get_button(axes_win.get('button_a')):
            print("auto_control")
            flag = 1
            while (flag==1):
                done = pygame_init(done)
                if auto_init_count == 0:
                    auto_init_count,bcap_man = auto_init(serial_port,auto_init_count,gbx, gby, gb_dist, tx, ty, tz, rx, ry, rz, LIDAR_dist1, LIDAR_dist2, debugM)
                Ctl_com = auto_control(serial_port)
                serial_port_out(serial_port, Ctl_com) # send the control data out
                time.sleep(waitTime)

                flag, print_count = go_back2_upper_level(flag, print_count) # go back to upper level by pressing "back"
                flag = manual_in_auto(Ctl_com, serial_port, flag) # turn into manual control in auto_control
                bcap_man = manual_ballcapture(bcap_man)    # manually determine ball capture status

        elif joystick.get_button(axes_win.get('button_LB')):
            print("manual control")
            flag = 2
            while (flag == 2):
                done = pygame_init(done)
                Ctl_com = manual_control()
                serial_port_out(serial_port, Ctl_com)
                flag, print_count = go_back2_upper_level(flag, print_count)

        elif joystick.get_button(axes_win.get('button_start')):
            print("test the camera core function")
            flag = 3
            while (flag == 3):
                done = pygame_init(done)
                test_function()
                flag, print_count = go_back2_upper_level(flag, print_count)

        elif joystick.get_button(axes_win.get('button_RB')):
            print("stop all motors")
            Ctl_com = stop_all()
            serial_port_out(serial_port, Ctl_com)

        pygame.time.wait(100)

        if print_count != 0:
            print("No subsystem is running")
            print_count = 0
        pygame.time.wait(100)

    pygame.quit()
