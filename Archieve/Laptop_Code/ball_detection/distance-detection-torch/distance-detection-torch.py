import os, glob
import json
import math
import torch
import numpy as np
import matplotlib.pyplot as plt
import xml.etree.ElementTree as gfg

from datetime import datetime
from mpl_toolkits import mplot3d

dtype  = torch.float
device = torch.device("cuda:0")

mode         = 'load-testFile'
stdFactor    = 2.0
learningRate = 4e-4
numIters     = 100000
mainDir      = 'G:/Shared drives/ECE 209  Blimp project/Images/train-test-3/'
inDir        = mainDir + 'train/'
testFile     = mainDir + 'test/' + '0277-01-1-300.xml'
weightsFile  = 'distance-detection-weights-3-2.0sd-20210808_212550.json'

def returnInput(inFile):
  tree   = gfg.parse(inFile)
  root   = tree.getroot()
  dims   = [int(val.text) for val in root[4][:2]]
  bndbox = [int(val.text) for val in root[6][4]]
  x = (bndbox[2]-bndbox[0])/dims[0]
  y = (bndbox[3]-bndbox[1])/dims[1]
  z = int(root[1].text[10:13])
  return [x,y,z]

def train(learningRate, numIters, inDir, numWts, specWts = []):
  x, y, z = [[] for i in range(3)]
  X, Y, Z = [{} for i in range(3)]
  for file in glob.glob(inDir + '*.xml'):
    inVal = returnInput(file)
    if inVal[2] in X:
      X[inVal[2]].append(inVal[0])
      Y[inVal[2]].append(inVal[1])
      Z[inVal[2]].append(inVal[2])
    else:
      X[inVal[2]] = [inVal[0]]
      Y[inVal[2]] = [inVal[1]]
      Z[inVal[2]] = [inVal[2]]
  for zVal in Z:
    xList = X[zVal]
    yList = Y[zVal]
    xMean = float(np.mean(xList))
    yMean = float(np.mean(yList))
    xStd  = float(np.std(xList))
    yStd  = float(np.std(yList))
    for count in range(len(xList)):
      xVal = xList[count]
      yVal = yList[count]
      if abs(xVal - xMean) < xStd*stdFactor and abs(yVal - yMean) < yStd*stdFactor:
        x.append(xVal)
        y.append(yVal)
        z.append(zVal)
  xT = torch.tensor(x, device=device, dtype=dtype)
  yT = torch.tensor(y, device=device, dtype=dtype)
  zT = torch.tensor(z, device=device, dtype=dtype)

  wts     = [torch.tensor(0.1, device=device, dtype=dtype, requires_grad=True) for i in range(numWts)]
  for pair in specWts:
    wts[pair[0]] =  torch.tensor(pair[1], device=device, dtype=dtype, requires_grad=True)
  for t in range(numIters):
    z_pred = genZ3(wts, xT, yT)
    loss = (z_pred - zT).pow(2).sum()
    if t % 1000 == 999:
      print(t, ' ', loss.item())
    loss.backward()
    with torch.no_grad():
      for count in range(len(wts)):
        wts[count] -= learningRate * wts[count].grad
      for count in range(len(wts)):
        wts[count].grad = None
  return wts

def genZ1(wts, x, y):
  z = wts[0]\
    + wts[1] *x       + wts[2]       *y\
    + wts[3] *x*x     + wts[4] *x    *y + wts[5]     *y*y\
    + wts[6] *x*x*x   + wts[7] *x*x  *y + wts[8] *x  *y*y + wts[9]   *y*y*y\
    + wts[10]*x*x*x*x + wts[11]*x*x*x*y + wts[12]*x*x*y*y + wts[13]*x*y*y*y + wts[14]*y*y*y*y
  return z

def genZ2(wts, x, y):
  z = wts[0]\
    + wts[1]*x\
    + wts[2]*x*x\
    + wts[3]*x*x*x\
    + wts[4]*x*x*x*x\
    + wts[5]*x*x*x*x*x\
    + wts[6]*x*x*x*x*x*x
  return z

def genZ3(wts, x, y):
  z = wts[0]\
    + wts[1] *x                                 + wts[2] *y\
    + wts[3] *x*x         + wts[4] *x*y         + wts[5] *y*y\
    + wts[6] *x*x*x       + wts[7] *x*x*y       + wts[8] *y*y*y\
    + wts[9] *x*x*x*x     + wts[10]*x*x*y*y     + wts[11]*y*y*y*y\
    + wts[12]*x*x*x*x*x   + wts[13]*x*x*x*y*y   + wts[14]*y*y*y*y*y\
    + wts[15]*x*x*x*x*x*x + wts[16]*x*x*x*y*y*y + wts[17]*y*y*y*y*y*y
  return z

def distanceDetect(mode, weightsFile = weightsFile, args = []):
  if 'load' in mode:
    with open(weightsFile, 'r') as f:
      wtsF = json.load(f)
  else:
    wts  = train(learningRate, numIters, inDir, 18, [(3, 300.0), (10, 300.0)])
    wtsF = [float(wt.cpu().data.numpy()) for wt in wts]
  if 'store' in mode:
    with open('distance-detection-weights-3-2.0sd-' + datetime.now().strftime("%Y%m%d_%H%M%S") + '.json', 'w') as f:
      json.dump(wtsF, f, indent = 2)
  if 'testFile' in mode:
    testFile = args[0]
    x, y, z = returnInput(testFile)
    Z = genZ3(np.array(wtsF), x, y)
    print("Estimated Distance = ", Z, ", Actual Distance = ", z)
  if 'plot' in mode:
    X = np.linspace(0, 1, 5000)
    Y = np.linspace(0, 1, 5000)
    X, Y = np.meshgrid(X, Y)
    Z = genZ3(np.array(wtsF), X, Y)
    fig = plt.figure()
    ax  = plt.axes(projection='3d')
    ax.contour3D(X, Y, Z, 400)
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')
    plt.show()
  if 'testImg' in mode:
    x0, x1, y0, y1 = args[0]
    W, H = args[1]
    x, y = (x1 - x0) / W, (y1 - y0) / H
    Z = genZ3(np.array(wtsF), x, y)
    return Z

if __name__ == '__main__':
  distanceDetect(mode, weightsFile, [testFile])
