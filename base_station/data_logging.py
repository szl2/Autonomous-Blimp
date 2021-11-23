import pickle
import logging
import matplotlib.pyplot as plt

def logger_setup():
    logger = logging.getLogger()
    logger.setLevel(logging.INFO)
    formatter = logging.Formatter('%(asctime)s | %(message)s')
    return logger, formatter

def fileHandler_setup(logger, formatter, fileName):
    fileHandler = logging.FileHandler(fileName)
    fileHandler.setLevel(logging.DEBUG)
    fileHandler.setFormatter(formatter)
    logger.addHandler(fileHandler)
    return logger, fileHandler

def log_variable(logger, varName = '', varValue = ''):
    if varName == '':
        logger.info('')
    while len(varName) < 36:
        varName += ' '
    if varValue == '':
        logger.info(str(varName))
    else:
        logger.info(str(varName) + ' | ' + str(varValue))

def log_image(folderName, frameName, frameTime, frame):
    file = open(folderName + frameName + '-' + frameTime + '.pkl', "wb")
    pickle.dump(frame, file)
    file.close()

def displayImage(fileName):
    file  = open(fileName, 'rb')
    image = pickle.load(file)
    plt.imshow(image)
    plt.show()

if __name__ == '__main__':
    print('Input pickle file name: ')
    pickleFile = input()
    displayImage(pickleFile)
