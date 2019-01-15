'''
Usage:  python md_readPCD.py numEpiInicial numEpiFinal 3D/2D

eg: python readAllPCD_xy.py 10 10 3D 
Will get only the 10th episode 

This code is utilized to read PCD files genereted by blensor and post-processing it, at the end generates a quantized
matrix of obstacles for each 'episode' for each receiver present in the episode. 

The point cloud data gets quantized utilizing the Quantization Parameters (QP)
where the area is delimited by Parameter_max and Parameter_min. Is advised to match these parameters to the delimitation parameters in
md_generateMatrixChannels.py. 

Inside the quantized matrix each point of obstacle is identified by 1, the Tx is identified by -1 and the Rx by -2

This process aims to fit all the data of the PCD files into a same shape of matrix
because the number of points returned by each scan is potentially different
also to decrease the number of points using quantization.

'''
import sys  
import os
import csv
import argparse
import sqlite3
import shutil
import numpy as np
#import sparse
import scipy
import scipy.spatial.distance as dist
import pypcd
from datetime import datetime
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
from scipy import ndimage

def main():
    startTime = datetime.now()

    print('Check Quantization parameters and Tx position before run!')
    fileToRead = 'matrixChannels.csv'
    if len(sys.argv) == 5:
        starting_episode = sys.argv[1]
        last_episode = sys.argv[2]
        type_data = sys.argv[3]
        scan_type = ''
        if int(sys.argv[4]) == 1:
            scan_type = '_noisy'
    else:
        print('Usage:  python ' + sys.argv[0] + ' numEpiInicial numEpiFinal 3D/2D noise(1)/noiseless(0)')
        exit(1)

    outputFolder = './obstacles_new_'+scan_type+type_data+'/'
    if not os.path.exists(outputFolder):
        os.makedirs(outputFolder)
    # Configuration of parameters
    dictvehicle = {1.59 : 5, 3.2 : 9.5, 4.3 : 13} #CarSize/BusSize/TruckSize
    # Quantization parameters
    QP = {'Xp':1.15,'Yp':1.25,'Zp':1,'Xmax': 767,'Ymax': 679, 'Zmax': 10, 'Xmin': 744,'Ymin': 429, 'Zmin': 0 } #X Y Z

    Tx = [746, 560, 4]
    max_dist_LIDAR = 100 # in meters

    dx = np.arange(QP['Xmin'],QP['Xmax'],QP['Xp'])
    dy = np.arange(QP['Ymin'],QP['Ymax'],QP['Yp'])

    if type_data == '3D':
        dz = np.arange(QP['Zmin'],QP['Zmax'],QP['Zp'])
        zeros_array = np.zeros((10, np.size(dx), np.size(dy), np.size(dz)), np.int8)

    else:
        zeros_array = np.zeros((10, np.size(dx), np.size(dy)), np.int8)


    obstacles_matrix_array = zeros_array
    #Assumes 50 scenes per episode and 10 Tx/Rx pairs per scene
    episodeID = ''
    sceneID = '' 
    tmpvar = 0
    dbname = './episodedata.db'

    with open(fileToRead) as csvfile:
        reader = csv.DictReader(csvfile)
        for row in reader:
            if int(row['EpisodeID']) < int(starting_episode):
                print('Jumping Episodes',row['EpisodeID'],starting_episode)
                continue
            print('Reading data for Episode ' + str(row['EpisodeID']) + '...')
            npz_name = outputFolder + 'obstacles_e_' + str(row['EpisodeID']) + '.npz'
            if os.path.exists(npz_name):
                print('Episode is already saved. Jumping to next...')
                continue
            scenes_path = findScenePathsDB(dbname,row['EpisodeID'],int(row['SceneID']))
            for scene_id, scene_path in scenes_path.items():
                tmpdir = '/tmp/scans'+ str(starting_episode) + '/'
                if (episodeID != row['EpisodeID'] and tmpvar == 1):
                    print('Episode is over. Time to save ObstaclesMatrix')
                    npz_name = outputFolder + 'obstacles_e_' + str(episodeID) + '.npz'
                    np.savez_compressed(npz_name, obstacles_matrix_array=obstacles_matrix_array)
                    print('Saved file ', npz_name)
                    obstacles_matrix_array = zeros_array

                if int(row['EpisodeID']) == int(last_episode) + 1:
                    return

                if (episodeID == row['EpisodeID'] and sceneID == row['SceneID']):
                    print('Scene: '+sceneID + ', Vehicle: '+ row['VehicleName'])
                else:
                   if os.path.exists(tmpdir):
                       shutil.rmtree(tmpdir)
                   else:
                       os.makedirs(tmpdir)
                   print('New scene!')
                   print('Scene: '+ str(row['SceneID']) + ', Vehicle: '+ row['VehicleName'])
                   episodeID = row['EpisodeID']
                   sceneID = row['SceneID'] 
                   pathdir =  scene_path.replace(' ','\ ') + '/scans' 
                   os.system(('unzip %s_%s.zip -d %s' %(pathdir,pathdir[-14:][:-6],tmpdir)))

                   vPosition = {}
                   v = getVertex(scene_path)
                   for vehicles in v.items():
                       vPosition[vehicles[0]] = calcPosition(vehicles[1]) 

                [selectedVehicles, sID] = selectVehicles_name(dbname,row['EpisodeID'],vPosition,row['VehicleName'])
 

                ###### LOOP para leitura de cada PCD
                tmpvar = 1
                for vehicle in selectedVehicles.items():
                    vehicle_position = [[vehicle[1][0],vehicle[1][1],vehicle[1][2]]] 
                    pcd_path = tmpdir + vehicle[0]  + scan_type + '00000.pcd' # if noisy add before zeros 'noisy' 
                    if not os.path.exists(pcd_path):
                        print('This vehicle: '+ vehicle[0] +' is not present in this scenario.')
                        print(pcd_path)
                        continue
                    
                    pc = pypcd.PointCloud.from_path(pcd_path)

                    #Filter1 : Removing Floor 
                    ind = np.where(pc.pc_data['z'] > 0.2)
                    fCloud = pc.pc_data[ind]
                    tmpCloud = [[i['x'], i['y'], i['z']] for i in fCloud]

                    #Filter2: Removing every obstacle bigger than max_dist_LIDAR
                    D = dist.cdist(vehicle_position,tmpCloud,'euclidean')
                    ind2 = np.where(D[0] < max_dist_LIDAR) # MaxSizeLIDAR
                    fffCloud = fCloud[ind2]


                    indx = quantizeJ(fffCloud['x'],dx)
                    indx = [int(i) for i in indx]
                    indy = quantizeJ(fffCloud['y'],dy)
                    indy = [int(i) for i in indy]

                    Rx_q_indx = quantizeJ([vehicle[1][0]],dx)
                    Rx_q_indy = quantizeJ([vehicle[1][1]],dy)
                    Tx_q_indx = quantizeJ([Tx[0]],dx)
                    Tx_q_indy = quantizeJ([Tx[1]],dy)

                    if type_data == '3D':
                        indz = quantizeJ(fffCloud['z'],dz)
                        indz = [int(i) for i in indz]
                        Rx_q_indz = quantizeJ([vehicle[1][2]],dz)
                        #Tx_q_indz = quantizeJ([10],dz)
                        Tx_q_indz = quantizeJ([Tx[2]],dz)
                        MD = np.zeros((np.size(dx),np.size(dy),np.size(dz)))
                    else:
                        MD = np.zeros((np.size(dx),np.size(dy)))

                    # Obstacles = 1
                    for i in range(len(indx)):
                        if type_data == '3D':
                            MD[indx[i],indy[i],indz[i]] = 1
                        else:
                            MD[indx[i],indy[i]] = 1

                    # Tx -1 Rx -2
                        if type_data == '3D':         
                            MD[int(Tx_q_indx[0]),int(Tx_q_indy[0]),int(Tx_q_indz[0])] = -1
                            MD[int(Rx_q_indx[0]),int(Rx_q_indy[0]),int(Rx_q_indz[0])] = -2
                        else:
                            MD[int(Tx_q_indx[0]),int(Tx_q_indy[0])] = -1
                            MD[int(Rx_q_indx[0]),int(Rx_q_indy[0])] = -2

                    obstacles_matrix_array[int(row['VehicleArrayID']), :] = MD 
                    time_elapsed = datetime.now() - startTime
                    print("Time elapsed: " + str(time_elapsed))



    npz_name = outputFolder + 'obstacles_e_' + str(episodeID) + '.npz'
    np.savez_compressed(npz_name, obstacles_matrix_array=obstacles_matrix_array)
    print('Saved file ', npz_name)
    time_elapsed = datetime.now() - startTime
    print("Total time elapsed: " + str(time_elapsed))

def quantizeJ(signal, partitions):
    xmin = min(signal)
    xmax = max(signal)
    M = len(partitions)
    delta = partitions[2] - partitions[1];
    quantizerLevels = partitions;
    xminq = min(quantizerLevels)
    xmaxq = max(quantizerLevels)
    x_i = (signal-xminq) / delta #quantizer levels
    x_i = np.round(x_i)
    ind = np.where(x_i < 0)
    x_i[ind] = 0
    ind = np.where(x_i>(M-1))
    x_i[ind] = M-1; #impose maximum
    x_q = x_i * delta + xminq;  #quantized and decoded output

    return list(x_i)

def findScenePathsDB(dbname,episode_id,scene_itr):
    conn = sqlite3.connect(dbname)
    cursor = conn.execute("SELECT id, insite_pah FROM episodes WHERE id=" + str(episode_id))
    scenes_path = {}
    for row in cursor:
        count = scene_itr - 1;
        allpath = row[1];
        run_base = allpath[-5:]
        tmp = int(run_base) + count
        scene_run = '%05d' %  tmp
        scenes_path[tmp+1] = allpath[:-5] + scene_run
    conn.close()
    return scenes_path



# Reads random-line.object and gets the vertex of the vehicles
def getVertex(filepath):  
    #filepath = '.' + filepath + "/random-line.object"
    filepath =  filepath + "/random-line.object"
    if not os.path.isfile(filepath):
        print("File path {} does not exist. Exiting...".format(filepath))
        sys.exit()
    with open(filepath) as fp:
        cnt,jump,read = 0,0,0
        vertex = {}
        tmp = []
        name = '';
        for line in fp:
            cnt += 1
            if(jump != 0):
                jump -= 1
                continue
            if(line.startswith("begin_<structure>")):
                name = line.strip().split(' ')[1]
                vertex[name] = []
                jump = 4
                read = 4
            elif (read != 0):
                tmp = (float(line.strip().split(' ')[0]),float(line.strip().split(' ')[1]),float(line.strip().split(' ')[2]))
                vertex[name] += [tmp]
                read -= 1
        #print(vertex)
        return vertex

# Return select vehicles based on the database
def selectVehicles_name(dbname,key_scene,vPosition,vName):
    conn = sqlite3.connect(dbname)
    fvPosition = {}
    fvID = {}
    cursor = conn.execute("SELECT objects.name as vehicle_name, scenes.id as scene_id, objects.id as vehicle_id FROM rays, receivers, objects, scenes, episodes WHERE episodes.id=scenes.episode_id AND rays.receiver_id = receivers.id AND receivers.object_id = objects.id AND objects.scene_id = scenes.id AND episodes.id ="+ str(key_scene) +" AND objects.name ='"+ str(vName)+"'  GROUP BY objects.name")
    for row in cursor:
        for i in vPosition.items():
            if(row[0] == i[0]): #row[0] -> name of vehicle
                fvPosition[row[0]] = i[1]
                fvID[row[0]] = row[2]
    conn.close()
    return fvPosition,fvID

# Calculates PolygonArea
def PolygonArea(corners):
    n = len(corners) # of corners
    area = 0.0
    for i in range(n):
        j = (i + 1) % n
        area += corners[i][0] * corners[j][1]
        area -= corners[j][0] * corners[i][1]
    area = abs(area) / 2.0
    return area

# Calculate position of a given vehicle
# Returns X Y Z
def calcPosition(verts):
    area = PolygonArea(verts)
    n = len(verts)
    x = 0.0
    y = 0.0
    z = 0.0
    for i in range(n):
        j = (i + 1) % n
        x += ( (verts[i][0] + verts[j][0]) * ((verts[i][0] * verts[j][1]) - (verts[j][0] * verts[i][1])) )
        y += ( (verts[i][1] + verts[j][1]) * ((verts[i][0] * verts[j][1]) - (verts[j][0] * verts[i][1])) )
    x = x / (6*area)
    y = y / (6*area)
    z = verts[0][2]
    return (x, y, z)

if __name__ == '__main__':
    main()
