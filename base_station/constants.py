BAUD_RATE   = 115200
MAX_SPEED   = 255
RESOLUTION_W = 800.0
RESOLUTION_H = 600.0
ENDDEMO_TAG  = 0
THISNODE_ID  = 88789821

### Ball Detection Constants
device            = 'cuda'
labelSet          = ['ball']
modelLoc          = './ball_detection/model_weights/'
modelFile         = 'model_weights-2-20210818_002355-cpu.pth'

minDetectionScore = 0.95

base_speed    = 50
seeking_speed = 120
LIDAR_Thres   = 500 # mm

# PID Control in move2ball
kpx,kix,kdx = 0.75, 0.01, 0.6
# kpx,kix,kdx = 0.0, 0.00, 0.0
kpy,kiy,kdy = 0.6, 0.01, 0.6#1.2, 0.02, 0.6
# kpy,kiy,kdy = 0.0, 0.0, 0.0

# PID Control in move2goal
# kpx_g,kix_g,kdx_g = 1.5, 0.01, 0.5
kpx_g,kix_g,kdx_g = 0.8, 0.01, 0.5
# kpx_g,kix_g,kdx_g = 0.0, 0.0, 0.0
kpy_g,kiy_g,kdy_g = 2.0, 0.01, 0.8#0.7
# kpy_g,kiy_g,kdy_g = 0.0, 0.0, 0.0

# difference between center of AT and center of goal
AT_goal_Delta =  0 #cm

AT_h1 = 140 # openmv: | esp32_cam:meters

kph,kih,kdh = 1.2,0.01,0.5

# Break between AT detection during ball seeking
AT_detectBreak = 60
#CONSTANTS NEEDED FOR GOAL DETECTION
leftSideAT = [1,2,5] #[1,2,3,4,5,6]
rightSideAT = [7,8,9,10,11,12]
leftSideOpponent = True #change this to false if teh opponent goal is the right side

#CONSTANTS NEEDED FOR THE SEEKING BEHAVIOR
rotate360Time = 20
moveUpTime = 8
moveDownTime = 10 #for the blimp to go down to approximately the middle of the court vertically

ml = 1
esp_cam_on = 1
openmv_on = 1
seekVertDir = 1

##VARIABLES NEEDED FOR HIGH LEVEL PLANNING
lookAheadPID_Ball = False
CloseBallSeekingTime = 10
CloseBallReverseTime = 5
CloseBallReverseSpeed = 200
ballHoverDistance = 400 #in meter
goFortheBall = True

lookAheadPID_Goal = False
goFortheGoal = True #Hover at a distance if FALSE
goalHoverDistance = 5 #in m
CloseGoalSeekingTime = 10 #in seconds
SpiralUpSeekingTime = 80 #in seconds 30
SpiralDownSeekingTime = 20 #in seconds 40
TrytoFindGoalAgainTime = 10

Rotate_VerticalSpeed = 220
Rotate_SpinningSpeed = 220
#LIST OF STATE
#MOVINGTOGOAL
#CLOSETOGOALSEEKING
#SPIRALUPGOALSEEKING
#SPIRALDOWNGOALSEEKING

#MOVINGTOBALL
#CLOSETOBALLSEEKING
#REGULARBALLSEEKING
#SPIRALUPABLLSEEKING
#SPIRALDOWNBALLSEEKING
