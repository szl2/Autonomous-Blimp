BAUD_RATE   = 115200
MAX_SPEED   = 255
RESOLUTION_W = 320.0
RESOLUTION_H = 240.0
ENDDEMO_TAG  = 0
THISNODE_ID  = 88789821

Setpointx = 400.0
Setpointy = 300.0

### Ball Detection Constants
device            = 'cuda'
labelSet          = ['ball']
modelLoc          = './ball_detection/model_weights/'
modelFile         = 'model_weights-2-20210818_002355-cpu.pth'

minDetectionScore = 0.90

### Color
# LAB: L[min] - L[max], A[min] - A[max], B[min] - B[max]
# Green (old)         = (30, 100, -68, -13, 30, 127)
# Green (new)         = (30, 100, -49, -22, 31, 127)
# Red                 = (30, 100, 127, 41, 127, 13)
# Green + Yellow Wall = (30, 100, -68, 2, 6, 127)
Lmin = 30
Lmax = 100
Amin = -49
Amax = -22
Bmin = 31
Bmax = 127
threshold = [Lmin, Lmax, Amin, Amax, Bmin, Bmax]

base_speed    = 120
seeking_speed =  50
LIDAR_Thres   = 300 # mm


# PID Control in move2ball
kpx,kix,kdx = 1.2, 0.01, 0.5
# kpx,kix,kdx = 0.0, 0.00, 0.0
kpy,kiy,kdy = 1.2, 0.05, 0.9
# kpy,kiy,kdy = 0.0, 0.0, 0.0

# PID Control in move2goal
# kpx_g,kix_g,kdx_g = 1.5, 0.01, 0.5
kpx_g,kix_g,kdx_g = 1.6, 0.01, 0.7
# kpx_g,kix_g,kdx_g = 0.0, 0.0, 0.0
kpy_g,kiy_g,kdy_g = 2, 0.01, 0.7
# kpy_g,kiy_g,kdy_g = 0.0, 0.0, 0.0

# difference between center of AT and center of goal
AT_goal_Delta =  0 #cm

AT_h1 = 140 # openmv: | esp32_cam:meters

kph,kih,kdh = 1.2,0.01,0.5

# Break between AT detection during ball seeking
AT_detectBreak = 60

url_AT = 'http://10.0.0.5/cam-hi.jpg'  # 1
url_gb = 'http://10.0.0.4/cam-hi.jpg'  # 6