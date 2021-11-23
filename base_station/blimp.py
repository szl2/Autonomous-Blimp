import time
import pygame
from data_logging import *
from websocket import create_connection

def direction_translation(dir1, dir2, dir3, dir4):
    new_dir1 = 0
    new_dir2 = 0
    new_dir3 = 0
    new_dir4 = 0


    if dir1 == '+':
        new_dir1 = 2
    else:
        new_dir1 = 1

    if dir2 == '+':
        new_dir2 = 1
    else:
        new_dir2 = 2

    if dir3 == '+':
        new_dir3 = 2
    else:
        new_dir3 = 1

    if dir4 == '+':
        new_dir4 = 1
    else:
        new_dir4 = 2

    spinning = ""
    vertical = ""

    return new_dir1, new_dir2, new_dir3, new_dir4

class Blimp:
    def __init__(self, logger):
        print("Address of Blimp object: ", id(self))
        self.logger = logger
        self.target = "ws://10.0.0.4:81"
        log_variable(self.logger, 'myBlimp.target', self.target)
        self.ESP_RECONNECT_MAXTRIES = 2
        log_variable(self.logger, 'myBlimp.ESP_RECONNECT_MAXTRIES', self.ESP_RECONNECT_MAXTRIES)
        self.connectionStatus = False
        log_variable(self.logger, 'myBlimp.connectionStatus', self.connectionStatus)
        try:
            self.connectionStatus = True
            self.ws = create_connection(self.target)
            log_variable(self.logger, 'myBlimp.connectionStatus', self.connectionStatus)
        except:
            self.connectionStatus = False
            self.try_reconnect()
        print("Connected")

    def try_reconnect(self):
        reconnect_try = 1;
        self.connectionStatus = False
        while not self.connectionStatus:
            if (reconnect_try > self.ESP_RECONNECT_MAXTRIES):
                print("Reconnect not possible: Giving up")
                log_variable(self.logger, 'Reconnect to ESP32 not possible')
                exit()
            print("Trying to reconnect");
            try:
                self.ws.close()
            except:
                pass
            try:
                self.ws = create_connection(self.target)
                self.connectionStatus = True
                log_variable(self.logger, 'myBlimp.connectionStatus', self.connectionStatus)
                return True
            except:
                self.connectionStatus = False
                log_variable(self.logger, 'myBlimp.connectionStatus', self.connectionStatus)
            reconnect_try += 1
            time.sleep(0.2)
        self.ws = create_connection(self.target)
        return True

    def get_LidarDist1(self):
        return 0

    def get_DebugM(self):
        return 'Testing'

    def set_BlimpMotors(self, pwm1, pwm2, pwm3, pwm4, dir1, dir2, dir3, dir4):
        # print("_______________________________________________")
        # print("pwm1,pwm2,pwm3,pwm4:{},{},{},{}".format(pwm1,pwm2,pwm3,pwm4))
        # print("dir1,dir2,dir3,dir4:{},{},{},{}".format(dir1,dir2,dir3,dir4))
        # print("_______________________________________________")

        vert = ""
        thrust = ""
        spin = ""

        if (pwm1 == 0):
            vert = "NONE"
        elif (dir1 == '+' and dir2 == '+'):
            vert = "UPWARD"
        elif(dir1 == '-' and dir2 == '-'):
            vert = "DOWNWARD"
        else:
            vert = "SHOULDNT HAPPEN!!!"

        if (pwm3 == 0 and pwm4 == 0):
            spin = "NONE"
            thrust = "NONE"
        if (dir3 == '+' and dir4 == '+'):
            thrust = "FORWARD"
            if (pwm3 > pwm4):
                spin = "TURNING RIGHT"
            elif (pwm3 == pwm4):
                spin = "NONE"
            else:
                spin = "TURNING LEFT"
        elif (dir3 == '-' and dir4 == '-'):
            thrust = "BACKWARD"
            if (pwm3 > pwm4):
                spin = "TURNING LEFT"
            elif (pwm3 == pwm4):
                spin = "NONE"
            else:
                spin = "TURNING RIGHT"
        elif (dir3 == '+' and dir4 == '-'):
            spin = "TURNING RIGHT"
            if (pwm3 > pwm4):
                thrust = "FORWARD"
            elif (pwm3 == pwm4):
                thrust = "NONE"
            else:
                thrust = "BACKWARD"
        elif(dir3 == '-' and dir4 == '+'):
            spin = "TURNING LEFT"
            if (pwm3 > pwm4):
                thrust = "BACKWARD"
            elif (pwm3 == pwm4):
                thrust = "NONE"
            else:
                thrust = "FORWARD"
            thrust = "NONE"

        print("motion of the blimp  spin: ", spin, "    thrust: ", thrust, "    vertical: ", vert)
        log_variable(self.logger, '[spin, thrust, vert]', [spin, thrust, vert])

        new_dir1, new_dir2, new_dir3, new_dir4 = direction_translation(dir1, dir2, dir3, dir4)
        pwm1 = min(255, pwm1)
        pwm2 = min(255, pwm2)
        pwm3 = min(255, pwm3)
        pwm4 = min(255, pwm4)
        cmd = [126, new_dir1, int(pwm1), new_dir2, int(pwm2), new_dir3, int(pwm3), new_dir4, int(pwm4), 0]
        print(cmd)
        try:
            self.ws.send_binary(bytes(cmd))
        except Exception as e:
            self.connectionStatus = False
            self.try_reconnect()
            print("Connected")

        pygame.time.wait(20)
