import cv2
import pygame

kpx,kix,kdx = 1,0.2,0.5


def auto_control():
    print("auto_control function")

def stop_all():
    print("stop_all function")

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

def manual_control(Ctl_com):

    if get_key("w"):
        val = start_speed
        Ctl_com = manual_command(val,val, 0, 0, "+","+","+","+")

    elif get_key("s"):
        val = start_speed
        Ctl_com = manual_command(val, val, 0, 0, "-", "-", "+", "+")

    if get_key("UP"):
        val = start_speed

        Ctl_com = manual_command(0,0, val, val, "+","+","+","+")
    elif get_key("DOWN"):
        val = start_speed
        Ctl_com = manual_command(0,0, val, val, "+","+","-","-")

    elif get_key("LEFT"):
        val = start_speed
        Ctl_com = manual_command(0,0, val, val, "+","+","-","+")

    elif get_key("RIGHT"):
        val = start_speed
        Ctl_com = manual_command(0,0, val, val, "+","+","+","-")

    return Ctl_com
    # print("manual_control function")

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

def dynamic_variable(str_name_v):

    global kpx,kix,kdx,start_speed
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

def variables_change_once():

    str_name = input("Enter your variable: ")
    dynamic_variable(str_name)

    # print("variables_change function")

def init():
    pygame.init()
    win= pygame.display.set_mode((400,400))

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

if __name__ == '__main__':
    global start_speed
    start_speed = 70
    Ctl_com = [0, 0, 0, 0, "+", "+", "+", "+"]
    flag = 0
    print_count = 1
    init()
    while True:

        if get_key('a'):
            flag = 1
            while (flag == 1):
                auto_control()

                cap = cv2.VideoCapture(0)
                ret, frame = cap.read()
                if not ret:
                    continue
                cv2.imshow("image", frame)

                flag, print_count = keyboard_stop(flag,print_count)
                if flag == 0:
                    cap.release()
                    cv2.destroyAllWindows()

        elif get_key('s'):
            stop_all()
            print("stop all motors")

        elif get_key('m'):
            flag = 2
            while (flag == 2):
                Ctl_command = manual_control(Ctl_com)
                pwm1, pwm2, pwm3, pwm4, dir1, dir2, dir3, dir4 = decode_ctl(Ctl_command)
                print("Ctl_com:{},{},{},{},{},{},{},{}".format(pwm1, pwm2, pwm3, pwm4, dir1, dir2, dir3, dir4))
                flag, print_count = keyboard_stop(flag,print_count)

        elif get_key('v'):
            flag = 3
            while (flag == 3):
                variables_change_once()
                flag = 0
                print_count = 1
                # flag, print_count = keyboard_stop(flag,print_count)
                
        elif get_key('k'): # kill the program
            break

        if print_count is not 0:
            print("No subsystem is running")
            print("kpx:{}".format(kpx))
            print("kix:{}".format(kix))
            print("kdx:{}".format(kdx))
            print_count = 0