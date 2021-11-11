# Laptop Code

## 1 Main structure 

├─ball_detection
│  ├─distance-detection-torch
│  │  └─__pycache__
│  ├─model_weights
│  └─__pycache__
├─basic_comm_test
│  ├─ESP32_master
│  └─ESP32_slave
├─ESP32_AT
│  └─__pycache__
├─ESP32_slave
├─Figs
├─main_backup
├─previous_high_level
├─Previous_low_level
│  ├─ESP32_master
│  └─ESP32_slave
├─simple_pid
│  └─__pycache__
├─system_description
├─Test_keyboard
├─Test_PID
│  └─test_pid
├─Two_Lidars_comm_test
│  ├─ESP32_master
│  └─ESP32_slave
└─__pycache__

## 2 Software Requirements

### Step 1: 

Download Anaconda or Pycharm

### Step 2: 

- If using Anaconda, then create a virtual environment first:

```shell
# crreate virtual environment
conda create -n FORAYenv python=3.7
# activate virtual environment
conda activate FORAYenv
# install environment installation tool pip (if it doesn't have one)
conda install pip
```

- If using Pycharm, at the very beginning, create a virtual environment

![](\Figs\fig1.png)

### Step 3:

Install dependency

- If using Anaconda, install it from the terminal. And always remember to activate the conda environment fist: `conda activate FORAYenv`

```shell
# install pyserial
pip install pyserial
# install pygame 
pip install pygame
# install cv2
pip install opencv-python
# install torch 
conda install pytorch torchvision torchaudio cudatoolkit=10.2 -c pytorch
# install pupil-apriltags
pip install pupil-apriltags
# install detecto
pip install detecto==1.2.1
```



## 3 Main_code

There are two files about main code: 

- constants.py: it stores global variables that can be changed easily

- main_keyboard: it has keyboard interruption and it contains all the functions that we need.

### 3.1 Flowchart

![](\Figs\flowchart_old.png)

### 3.2 Functions name and their inputs outputs

All function: 

```
init()
auto_init()
auto_control()
	main_control()
	ball_detect()
	ball_capture()
	goal_detect()
	move2goal()
	rotate_one_direction()
	move2ball()
	
serial_port_out()

```



```python
init()
    """
    Description:

    Input:
        no 
    Output:
        no 
    """
```

```python
auto_init()
"""
    Description:
		
    Input:
        no 
    Output:
        no 
"""
```

