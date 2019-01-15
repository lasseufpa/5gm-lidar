import os
import csv
import numpy as np
import sys

def getInfo(filename,inputPath):
    with open(filename) as csvfile:
        reader = csv.reader(csvfile)
        allLines = list(reader)
        numExamples = len(allLines) - 1 # header

    currentEpisodesInputs = np.load(os.path.join(inputPath,'obstacles_e_10.npz'))
    obstacles_matrix_array = currentEpisodesInputs['obstacles_matrix_array']
    return numExamples,obstacles_matrix_array.shape


filename = 'matrixChannels.csv'
argv = sys.argv
inputPath = argv[1]
npz_name = argv[2]

numExamples,input_shape = getInfo(filename,inputPath)
with open(filename) as csvfile:
    reader = csv.DictReader(csvfile)
    allInputs = np.zeros((numExamples,input_shape[1],input_shape[2],input_shape[3]), np.int8)
    id_count = 0
    alreadyInMemoryEpisode = -1
    if os.path.exists(npz_name):
        print( npz_name, 'already exists')
        exit(1)
    for row in reader:
        episodeNum = int(row['EpisodeID'])
        #if (episodeNum < numEpisodeStart) | (episodeNum > numEpisodeEnd):
        #    continue #skip episodes out of the interval
        isValid = row['Val'] #V or I are the first element of the list thisLine
        if isValid == 'I':
            continue #skip invalid entries
        if episodeNum != alreadyInMemoryEpisode: #just read if a new episode
            print('Reading Episode '+str(episodeNum)+' ...')
            currentEpisodesInputs = np.load(os.path.join(inputPath,'obstacles_e_'+str(episodeNum)+'.npz'))
            obstacles_matrix_array = currentEpisodesInputs['obstacles_matrix_array']
            alreadyInMemoryEpisode = episodeNum #update for other iterations
        s = int(row['SceneID']) - 1 #get scene number
        r = int(row['VehicleArrayID']) #get receiver number
        allInputs[id_count] = obstacles_matrix_array[r]
        id_count = id_count + 1
    np.savez_compressed(npz_name, input_classification=allInputs)
    print('Saved file ', npz_name)

#if __name__ == '__main__':
#    runAll()
