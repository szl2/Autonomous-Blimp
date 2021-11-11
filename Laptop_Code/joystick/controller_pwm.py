import pygame

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
        dir4 = "-"

    return abs(pwm3),abs(pwm4),dir3,dir4

def manual_control():
    pwm1,pwm2,dir1,dir2 = manual_vertical()
    pwm3,pwm4,dir3,dir4 = manual_horizontal()
    print("pwm1,pwm2,pwm3,pwm4:{},{},{},{}".format(pwm1,pwm2,pwm3,pwm4))
    print("dir1,dir2,dir3,dir4:{},{},{},{}".format(dir1,dir2,dir3,dir4))
    print("_________________________________")

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

def go_back2_uper_level(flag_b,print_count_b):
    if joystick.get_button(axes_win.get('button_back')):
        flag_b = 0
        print_count_b = 1
    return flag_b,print_count_b

def pygame_init(done):
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            done = True
    return done
if __name__ == '__main__':
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

    flag = 0
    print_count = 0

    controller_id = 1
    while not done:

        done = pygame_init(done)

        joystick = pygame.joystick.Joystick(controller_id)
        joystick.init()

        if joystick.get_button(axes_win.get('button_start')) and joystick.get_button(axes_win.get('button_back')):
            print("kill the program")
            done = True

        elif joystick.get_button(axes_win.get('button_a')):
            print("auto_control")

        elif joystick.get_button(axes_win.get('button_RB')):
            print("stop all motors")

        elif joystick.get_button(axes_win.get('button_LB')):
            print("manual control")
            flag = 2
            while (flag == 2):
                done = pygame_init(done)
                manual_control()
                flag, print_count = go_back2_uper_level(flag, print_count)

        elif joystick.get_button(axes_win.get('button_start')):
            print("test the camera core function")

        pygame.time.wait(100)

        if print_count != 0:
            print("No subsystem is running")
            print_count = 0
        pygame.time.wait(100)

    pygame.quit()