'''
Create 4 files with LIDAR input matrices and beam pairs outputs, for training and testing.
Train and test have distinct episodes.
From
https://jakevdp.github.io/PythonDataScienceHandbook/04.12-three-dimensional-plotting.html
'''
import numpy as np
import csv
import os
import numpy as np
from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.interpolate import interp2d
from matplotlib import cm
import math
#from keras.utils import plot_model
import matplotlib.pyplot as plt
from scipy import ndimage
import scipy
#from PIL import Image  #used pip install pillow

def runAll():
    #list from which the examples are obtained
    inputListName = 'list2_only_valids'
    fileName = 'D:/github/5gm-data/' + inputListName + '.csv'
    #output npz file
    npz_name = inputListName + '.npz'

    with open(fileName, 'r') as f:
        reader = csv.reader(f)
        allLines = list(reader)
    #episodes start from 1 (not 0)
    #composeTrainTest(1, 119, allLines, 'notvalid') #numberOfValids =  41496 in this case
    #composeTrainTest(1, 1, allLines, 'notvalid')
    topKtest = 2 #use 1 for regular classifier and > 1 for top-k
    writeOutputs(allLines, npz_name, topKtest=topKtest)

def writeOutputs(allLines, npz_name, topKtest=1):
    if os.name == 'nt':
        inputPath = 'D:/github/5gm-data/outputnn/'
    else:
        inputPath = '/mnt/d/github/5gm-data/outputnn/'

    #number of selected codewords after running upa_codebook_prune_unused.m
    number_Tx_vectors = 20
    number_Rx_vectors = 12

    numExamples = len(allLines)
    #allocate space for all entries
    if topKtest == 1:
        allOutputs = np.zeros(numExamples, np.int8)
    else: #it's not simple classification but top-k
        allOutputs = np.zeros((numExamples,number_Rx_vectors,number_Tx_vectors), np.complex128)
    count = 0
    alreadyInMemoryEpisode = -1
    currentEpisodesInputs = None
    currentEpisodesOutputs = None
    #go over all lines, sequentially
    for lineNum in range(0, numExamples): #recall that we do not have a header
        thisLine = allLines[lineNum]
        episodeNum = int(thisLine[0])
        if episodeNum != alreadyInMemoryEpisode: #just read if a new episode
            print('Reading Episode '+str(episodeNum)+' ...')
            #sum 1 to episodeNum below because file names start from 1 while lists use indices starting from 0
            currentEpisodesOutputs = np.load(os.path.join(inputPath, 'output_e_'+str(episodeNum)+'.npz'))
            output = currentEpisodesOutputs['output']
            alreadyInMemoryEpisode = episodeNum #update for other iterations

        s = int(thisLine[1]) #get scene number
        r = int(thisLine[2]) #get receiver number

        if np.sum(np.isnan(output[s,r][:])) > 0:
            print('There is some error in the mapping between lists and files.')
            print('Found Nan! Stopping...')
            exit(-1)

        allOutputs[count] = output[s,r]
        count += 1
        #print('isNan = ', np.sum(np.isnan(allOutputs[:])))

    print('Sanity check (must be 0 NaN) sum of isNaN = ', np.sum(np.isnan(allOutputs[:])))
    np.savez(npz_name, output_classification=allOutputs)
    print('Saved file ',npz_name)

if __name__ == '__main__':
    runAll()
