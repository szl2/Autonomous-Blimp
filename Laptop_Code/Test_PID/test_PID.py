import sys
sys.path.append('../')
from simple_pid import PID
import time

if __name__ == "__main__":
    kdx,kix,kpx = 2,0.1,0.25
    kdy,kiy,kpy = 1,0.1,0.25
    
    setpoint_x = 160
    setpoint_y = 120
    inputx = 200
    inputy = 150
    pid_x = PID(kdx,kix,kpx,setpoint=setpoint_x)
    pid_y = PID(kdy,kiy,kpy,setpoint=setpoint_y)
    pid_x.auto_mode = True
    pid_x.set_auto_mode(True, last_output=8.0)
    pid_x.output_limits = (-255,255)
    pid_y.output_limits = (-255,255)
    """
    # to see how each of the components contribute to the output
    p, i, d = pid_x.components 
    c_p = pid_x.Kp
    c_i = pid_x.Ki
    c_d = pid_x.Kd
    """
    count = 0
    while True:
        if (count < 30):
            outputx = pid_x(inputx)
            outputy = pid_y(inputy)
            print("outputx:{}".format(outputx))
            print("outputy:{}".format(outputy))
        
        if (count > 30 and count < 60):
            pid_x.setpoint = 400
            pid_y.setpoint = 300
            inputx = 200
            inputy = 150
            outputx = pid_x(inputx)
            outputy = pid_y(inputy)
            print("outputx:{}".format(outputx))
            print("outputy:{}".format(outputy))            
        count +=1
        time.sleep(0.1) # seconds
    
    