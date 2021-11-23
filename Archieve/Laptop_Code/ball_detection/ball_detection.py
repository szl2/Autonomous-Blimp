## Codebase - Live Training-based Ball Detection
### Import Modules and Drive
import os
import cv2
import glob
import torch
import numpy as np
import torch.nn as nn
import torch.quantization
import matplotlib.pyplot as plt
import matplotlib.patches as patches

from urllib.request import urlopen, Request  #

from base64 import b64decode
from datetime import datetime
from matplotlib import patches
from detecto import core, utils
from torchvision import transforms
from matplotlib import pyplot as plt
distanceDetect = __import__('ball_detection.distance-detection-torch.distance-detection-torch', fromlist = ['distanceDetect']).distanceDetect
# print("torch.cuda.is_available() = ", torch.cuda.is_available())

# change the IP address below according to the IP shown in the Serial monitor of Arduino code
# url='http://192.168.1.107/cam-lo.jpg'
# url='http://192.168.4.1/cam-hi.jpg'
# url='http://192.168.1.107/cam-mid.jpg'
# url='http://192.168.0.204/cam-hi.jpg' # url2

#### Modify Detecto Core to include possibility of other base models
def modifyCore():
  cModLineNums = [221, 253, 254]
  cAddLineNums = [254, 256, 257, 258, 259]

  # REPLACABLE LINE FOR DIFFERENT LOCAL COMPUTER DEVICES
  coreFile     = 'C:\\Users\\aaronjs\\.conda\\envs\\FORAYenv\\Lib\\site-packages\\detecto\\core.py'

  cModLineVals = ["    def __init__(self, classes=None, device=None, pretrained=True, modelname=\'fasterrcnn_resnet50_fpn\'):\n",
                  "        # Load a model pre-trained on COCO - User-Modified\n",
                  "            self._model = torchvision.models.detection.fasterrcnn_resnet50_fpn(pretrained=pretrained)\n"]
  cAddLineVals = ["        if modelname == \'fasterrcnn_resnet50_fpn\':\n",
                  "        elif modelname == \'fasterrcnn_mobilenet_v3_large_fpn\':\n",
                  "            self._model = torchvision.models.detection.fasterrcnn_mobilenet_v3_large_fpn(pretrained=pretrained)\n",
                  "        elif modelname == \'fasterrcnn_mobilenet_v3_large_320_fpn\':\n",
                  "            self._model = torchvision.models.detection.fasterrcnn_mobilenet_v3_large_320_fpn(pretrained=pretrained)\n",
                  "        else:",
                  "            return ValueError('Unknown Pretrained Model')"]
  coreRead = open(coreFile, "r")
  coreLines = coreRead.readlines()
  if coreLines[253][-14:-1] != 'User-Modified':
    for count, cModLineNum in enumerate(cModLineNums):
      coreLines[cModLineNum] = cModLineVals[count]
    for count, cAddLineNum in enumerate(cAddLineNums):
      coreLines.insert(cAddLineNum, cAddLineVals[count])
    coreRead.close()
    coreWrite = open(coreFile, "w")
    coreWrite.writelines(coreLines)
    coreWrite.close()

def modelName(type = 1):
  if type == 1:
    return 'fasterrcnn_resnet50_fpn'
  elif type == 2:
    return 'fasterrcnn_mobilenet_v3_large_fpn'
  elif type == 3:
    return 'fasterrcnn_mobilenet_v3_large_320_fpn'
  else:
    return ValueError('Unknown Pretrained Model')

def modelTitle(modelLoc, device, type):
  time = datetime.now().strftime("%Y%m%d_%H%M%S")
  if device == 'cpu':
    dev = 'cpu'
  elif device[:4] == 'cuda':
    dev = 'cuda'
  return modelLoc + 'model_weights-' + str(type) + '-' + time + '-' + dev + '.pth'

def returnModel(device, labelSet, modelLoc, modelFile):
  model = core.Model(labelSet, modelname = modelName(int(modelFile[14])))
  model._model.load_state_dict(torch.load(modelLoc + modelFile, map_location = torch.device(device)))
  return model

#### Live Detection
def detectLive(url , model, minDetectionScore = 0.90, showSight = True):
    '''
    Descrption:
        ball detection ML Functions

    Input:
        the ML modelLoc

    Output:
        -1 means not found
        otherwise return the actual ball information
    '''

    gbx  = -1
    gby  = -1
    dist = -1

    header = {"User-Agent": "Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/92.0.4515.159 Safari/537.36."}
    req      = Request(url, headers = header)
    img_resp = urlopen(req, timeout = 60)
    imgnp    = np.array(bytearray(img_resp.read()), dtype = np.uint8)
    frame    = cv2.imdecode(imgnp, -1)

    labels, boxes, scores = model.predict(frame)
    for i in range(boxes.shape[0]):
      box = boxes[i]
      x0, y0 = float(box[0]), float(box[1])
      x1, y1 = float(box[2]), float(box[3])
      w      = x1 - x0
      h      = y1 - y0
      dCrop  = [int(y0*cropFactor + y1*(1-cropFactor)), int(y1*cropFactor + y0*(1-cropFactor)),
                int(x0*cropFactor + x1*(1-cropFactor)), int(x1*cropFactor + x0*(1-cropFactor))]
      avgClRGB = cv2.mean(frame[dCrop[0]:dCrop[1], dCrop[2]:dCrop[3]])
      avgClHue = rgbToHue(avgClRGB)

      if scores[i] > minDetectionScore and avgClHue > colorLow and avgClHue < colorHigh:
        cv2.rectangle(frame, (int(x0), int(y0)),
                             (int(x1), int(y1)),
                             (0, 255, 0), 2)
        if labels:
          dist = distanceDetect('load-testImg', distWtsFile, [[x0, x1, y0, y1], frame.shape[:2]])
          gbx = int((x1 + x0) / 2)
          gby = int((y1 + y0) / 2)
          cv2.putText(frame, 'x:{}, y:{}'.format(gbx, gby),
                             (int(box[0]), int(box[1]) - 10),
                             cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 3)

    if showSight:
      cv2.imshow('Ball Detection', frame)
      key = cv2.waitKey(1) & 0xFF

    dist = int(dist)
    return gbx, gby, dist

### Supplementary Functions
#### Display of Image with Bounding Boxes
def displayBoxedImage(image, boxes = [], labels = None):
  fig, ax = plt.subplots(1)
  if isinstance(image, torch.Tensor):
    image = utils.reverse_normalize(image)
    image = transforms.ToPILImage()(image)
  ax.imshow(image)
  if labels is not None and not utils._is_iterable(labels):
    labels = [labels]
  for i in range(len(boxes)):
    box = boxes[i]
    width, height =  box[2] - box[0], box[3] - box[1]
    initial_pos   = (box[0], box[1])
    rect = patches.Rectangle(initial_pos, width, height, linewidth = 1, edgecolor = 'r', facecolor = 'none')
    if labels:
      ax.text(box[0] + 5, box[1] - 5, '{}'.format(labels[i]), color='red')
    ax.add_patch(rect)
  plt.show()

#### Calculation of Hue from RGB (adopted from [code](https://www.geeksforgeeks.org/program-change-rgb-color-model-hsv-color-model/))
def rgbToHue(RGB):
  RGB = [val/255.0 for val in RGB]
  cmax = max(RGB)
  cmin = min(RGB)
  diff = cmax - cmin
  r, g, b = RGB[0:3]
  if cmax == cmin:
    return 0
  if cmax == r:
    return (60 * ((g - b) / diff) + 360) % 360
  if cmax == g:
    return (60 * ((b - r) / diff) + 120) % 360
  if cmax == b:
    return (60 * ((r - g) / diff) + 240) % 360

#### Formula-based Depth Estimation (adopted from [mono_depth.py](https://github.com/agrawalparth10/FORAY-Perception/blob/master/mono_depth/mono_depth.py))
def depth_estimate(figDims, boundingBox):
  def x_hat(realLen, ip, i0, imageLen):
    return (realLen * (ip-i0)) / imageLen
  def y_hat(realLen, jp, j0, imageLen):
    return (realLen * (jp-j0)) / imageLen
  def z_hat(dist, pixelLen, imageLen):
    return dist*(pixelLen/imageLen)
  def cal_coordinates(dist, realLen, ip, jp, i0, j0, imageLen, pixelLen):
    return (x_hat(realLen, ip, i0, imageLen), y_hat(realLen, jp, j0, imageLen), z_hat(dist, pixelLen, imageLen))
  def measure(dist, realLen, ip, jp, i0, j0, imageLen, pixelLen):
    x_cor = x_hat(realLen, ip, i0, imageLen)
    z_cor = z_hat(dist, pixelLen, imageLen)
    dist  = (x_cor ** 2 + z_cor ** 2) ** 0.5
    return dist
  imHeight, imWidth = figDims
  center = (imWidth // 2, imHeight // 2)
  x, y, w, h   = boundingBox[0:4]
  relativeDist = measure        (dist = 0.51, realLen = 0.228, ip = x, jp = y, i0 = center[0], j0 = center[1], imageLen = w, pixelLen = 500)
  coordinates  = cal_coordinates(dist = 0.51, realLen = 0.228, ip = x, jp = y, i0 = center[0], j0 = center[1], imageLen = w, pixelLen = 500)
  return relativeDist, coordinates

### Declare Varables and Constants
colorLow    =  60
colorHigh   = 180
cropFactor  =   0.90
distWtsFile = './ball_detection/distance-detection-torch/distance-detection-weights-3-2.0sd-20210808_212550.json'
