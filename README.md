Tasks that can be performed with the LIDAR and 5gm-data softwares:
=============

1) How to design a new simulation with SUMO and InSite

2) How to run the BlenSor simulation after having InSite results

3) How to convert InSite into MIMO channels

4) How to convert Blensor output into obstacle matrices

5) Using Python to process MIMO channels and generate beam-selection outputs

6) Beam-selection using LIDAR data

7) Beam-selection using the positions of the vehicles

8) Use LIDAR data for LOS detection

We detail each one below.


1)How to design a new simulation with SUMO and InSite
=============

TBD

InSite results (https://owncloud.lasseufpa.org/s/SgxUXTi9niGw915) - 1.1 GB  

2)How to run the BlenSor simulation after having InSite results
============

Download LIDAR data, from 
Blensor results (https://owncloud.lasseufpa.org/s/9TzYsxYrIrgPrP4) - 57.9 GB

3)How to convert InSite into MIMO channels
============
Here we assume that both InSite and Blensor were already executed and show steps to organize the data.
Assume the InSite data is at folder 
D:\insitedata\noOverlappingTx4m_s1130 (where 1130 is the SUMO random seed). Another alternative is to name as
d:\insitedata\simul45_20180915  (format is year, month, day).
We have the following subtasks:
1. Channel generation: Convert InSite channel data in a database (episode.db) and also into files that can be easily read by Python (npz) and Matlab (hdf5)
2. Lists: Create two lists of text files. The first indicates all valid and invalid receivers, while the second deals only with the valid receivers and also informs the LOS and NLOS ones

3.1) - Channel generation
===========

1. First we need the episode.db corresponding to the RT simulations

cd D:\github\5gm-rwi-simulation

Consider deleting any file named episode.db in the current folder. 

In other words, check if you have a file name episode.db in this (current) folder. If you have, consider that the script todb.py appends the new data into an existing episode.db  file. So, you may want to move it to another folder or delete it in case you want to generate a new file, from scratch.

python todb.py D:\insitedata\noOverlappingTx4m_s1130

In the end you know how many episodes were processed and should take note of that because it will be requested, e.g.
```
Processed episode: 2086 scene: 1, total 2086
Warning: could not find file  D:\insitedata\noOverlappingTx4m_s1130\run02086\study\model.paths.t001_01.r002.p2m  Stopping...

Processed  2086  scenes (RT simulations)
```
In the case above there was N=1 scene per episode, but in general there are more than 1 scene per episode (N>1).

A file episode.db was created in current folder. Rename and move it to its final folder.

2. Write the ray information as both npz and hdf5 files. Write to folder ./insitedata. Note that all data is written and if the Tx / Rx pair is not valid, NaN are used.

First you need to edit and change variables such as numScenesPerEpisode.

cd D:\github\5gm-data

Copy the episode.db to the current folder. You may have different .db files:
```
D:\github\5gm-data>dir *.db
 O volume na unidade D é Data
 O Número de Série do Volume é 1CBF-8747

 Pasta de D:\github\5gm-data

19/09/2018  17:55       189.898.752 episodedata.db
19/09/2018  17:55       189.898.752 episodedata_correct_tx4m.db
11/06/2018  23:34       568.399.872 episodedata_e116.db
23/07/2018  16:53       382.160.896 episodedata_e119.db
12/09/2018  23:25       225.923.072 episodedata_flat_new_marcus.db
10/09/2018  23:20       162.029.568 episodedata_longepisodes.db
15/08/2018  11:47       189.792.256 episodedata_no_cir_overlap_tx4m.db
12/08/2018  02:40       143.196.160 episodedata_no_cir_overlap_tx5m.db
15/09/2018  17:56       225.796.096 episodedata_simul45_flat_new_marcus.db
```

Make sure you are using the right episode.db and edit convert5gmv1ToChannels.py
to indicate e.g. the output folder. If it is .\insitedata you may want to delete
all files before creating new (old files would be overwritten but if there are
less new files than older, you may mix files from 2 distinct simulations in
the same folder):

del .\insitedata\*
mkdir .\insitedata

python convert5gmv1ToChannels.py

Recall that episode.db knows how many rays were obtained (sometimes a number smaller than the maximum of e.g. 25 rays), and
the arrays saved by the script will use NaN when the number of rays is smaller than the maximum. 
One can always check the number of rays by looking at the two lists that will be created later.
At the end, you will find .hdf5 for Matlab and .npz for Python in the output folder and
be able to see how many LOS, NLOS and total valids (Sum) one has:
```
Start time =  62580000  and sampling period =  0.1  seconds
Episode: 2085 out of 2086
['flow7.5020', 'flow7.5021', 'flow7.5022', 'flow7.5023', 'flow8.2489', 'flow8.2493', 'flow9.2527', 'flow9.2528', 'flow9.2529', 'flow9.2531']
 Done: 100.00% Scene: 1 time per scene: 0:06:09.742866 time to finish: 0:00:00
==> Wrote file ./insitedata/urban_canyon_v2i_5gmv1_rays_e2085.npz
==> Wrote file ./insitedata/urban_canyon_v2i_5gmv1_rays_e2085.hdf5
numLOS =  6482
numNLOS =  4712
Sum =  11194
```

3.2) Create 4 lists
===========

We will use only four lists as text (in fact CSV) files. We show how to generate them.

3. Write the 1st list by redirecting the stdout with > 

cd D:\github\5gm-data

python generateInfoList.py > list1_valids_and_invalids.csv

Note that the output will depend on the episode.db that is in the current folder.
You then need to edit this csv file to eliminate its first and last rows (that should be similar to the ones below):

```
First lines:
########## Important ##########
Will try to open database in file (should be in your current folder):  episodedata.db
Successfully opened  episodedata.db

Last lines:
numValidChannels =  11194
numInvalidChannels =  9666
Sum =  20860
numLOS =  6482
numNLOS =  4712
Sum =  11194
```

5.	Then we need a second list, which includes information only for the valid receivers

python generateInSitePlusSumoList.py D:\insitedata\noOverlappingTx4m_s1130 > list2_only_valids.csv

where the first argument (argv[1]) is the InSite output folder.
Then we can specialize the list of valid receivers to other lists with only LOS and NLOS:
```
grep LOS=0 list2_only_valids.csv > list2_only_validsNLOS.csv
grep LOS=1 list2_only_valids.csv > list2_only_validsLOS.csv
```

If you have wc (you are using e.g. Linux), you can check if the results below
match numLOS and numNLOS indicated in list1_valids_and_invalids.csv (see above):

```
wc *.csv
  20860   20860 1255435 list1_valids_and_invalids.csv
  11194   11194 1282362 list2_only_valids.csv
   6482    6482  741028 list2_only_validsLOS.csv
   4712    4712  541334 list2_only_validsNLOS.csv
  43248   43248 3820159 total
```

Now you have the channels and the 4 lists. You can continue use Python to do
beam-selection or work with LIDAR PCDs with Matlab. We assume the latter case in the next section.

4)How to convert Blensor output into obstacle matrices
===========

This process is Similar to channel generation using the insite files, but instead it will generate the so-called obstacle matrices, using the LIDAR output from blensor.

1. First we need the episode.db corresponding to the RT simulations

Make sure you are using the right episode.db and run generateMatrixChannels.py
```
python genereateMatrixChannels.py > matrixChannels.csv

```
Note that the output will depend on the episode.db that is in the current folder.
You then need to edit this csv file to eliminate its first and last rows (that should be similar to the ones below):

```
First lines:
########## Important ##########
Will try to open database in file (should be in your current folder):  episodedata.db
Successfully opened  episodedata.db

Last lines:
numValidChannels =  11194
numInvalidChannels =  9666
Sum =  20860
numLOS =  6482
numNLOS =  4712
Sum =  11194
```

Also, make sure to add to the created csv above the following header:
```
Val,EpisodeID,SceneID,VehicleArrayID,VehicleName,x,y,LOS
```
Note that if you download the Blensor files they should be inside a folder in their zips, you must run lnScans.py
to create a symbolic link of these files inside the inSite folder using lnScans.py script

 
```
5gm-lidar>python lnScans.py insitefolder allscansfolder
```

Now the obstacles matrices can be created using the md\_readPCD.py script. Note that this scripts utilizes python2 instead of python3

```
#You must choose the start and end episodes 
#3D will create 3D matrices (x,y,z) and 2D will created 2D matrices (x,y)
#0 - to utilize the noiseless LIDAR PCD data and 1 to use the noise data.

5gm-lidar>python readPCD.py epi_begin epi_end 3D/2D 1/0
```
After running the script above, you should have a folder such as 'obstacles\_new\_3d' with the obstacle matrices inside npz files organized per episode.


5)Using Python to process MIMO channels and generate beam-selection outputs
============

1.	Codebook design
	
The codebooks are generated with Matlab (very ugly code, should be later
ported to Python in clean rewrite)
```
D:\github\5gm-lidar\matlab\upa_codebook_creation.m
```
Assume we ran it twice to generate:
```
>> upa_codebook_creation
Wrote upa_codebook_16x16_N832.mat
>> upa_codebook_creation
Wrote upa_codebook_4x4_N52.mat
```

2.	Generating the equivalent channel gains that depend on the beams:

Assuming the convert5gmv1ToChannels.py already wrote the output files (e.g. in folder .\insitedata), then 
edit getBestBeamsFromChannelRays.py to change:

```
insiteCSVFile = 'D:/github/5gm-data/list2_only_valids.csv'
txCodebookInputFileName = 'D:/github/5gm-lidar/matlab/upa_codebook_16x16_N832.mat'
rxCodebookInputFileName = 'D:/github/5gm-lidar/matlab/upa_codebook_4x4_N52.mat'
numEpisodes = 2086  # total number of episodes
outputFolder = 'D:/github/5gm-data/outputnn/'
```

Make sure the output folder (e.g. D:\github\5gm-data\outputnn) does not contain files from previous simulations:
```
del outputnn\*
```
Note that two files will be written in the output folder: a) output_e_XXX.npz and b) outputs_positions_e_XXX.hdf5.
The first one is npz while the second is hdf5. Another difference is that while the first has only the array 
episodeOutputs with the equivalent channels per beam pair, the hdf5 have array episodeOutputs and array receiverPositions
with the positions of all receivers.

Evaluate the beams:
```	
D:\github\5gm-data>python getBestBeamsFromChannelRays.py
```

This script informs when it ends some statistics and the histogram for each codebook (Tx and Rx). Confirm the statistics match your numLOS and numNLOS
informed previously, and copy the histogram values to place on the Matlab script that prunes (reduces the sizes) of the codebooks.

Copy the histograms to script upa_codebook_prune_unused
```
tx_indices_histogram = [ 0 0 0 0 0 0 0 0 82 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 403 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 336 0 0 0 0 0 0 0 0 0 0 0 0 0 0 190 1 591 0 0 0 0 0 0 0 0 0 0 0 0 2 396 0 788 1 0 0 0 0 0 0 0 0 0 0 2 283 37 0 67 455 2 0 0 0 0 0 0 0 1 3 168 55 0 0 0 95 228 3 0 0 0 0 1 12 83 98 10 0 0 0 0 0 5 126 84 7 0 70 91 63 12 0 0 0 0 0 0 0 0 0 7 78 87 0 0 0 16 15 0 0 0 0 0 0 0 8 3 0 0 0 0 0 0 0 0 0 0 0 0 16 25 0 0 0 0 0 0 0 0 0 0 0 0 0 19 93 0 0 0 0 0 0 0 0 0 0 0 0 0 0 17 0 0 0 0 0 0 0 0 0 0 0 0 0 7 0 9 0 0 0 0 0 0 0 0 0 0 0 0 0 0 4 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 10 3 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 230 3 0 0 0 0 0 216 0 0 0 0 0 0 0 55 4 201 2 0 0 0 0 381 0 0 0 0 0 1 0 13 0 21 132 5 1 0 0 218 0 0 0 0 1 56 0 0 0 0 13 157 7 0 0 612 0 0 0 0 79 18 0 0 0 0 0 13 369 6 0 777 0 0 0 0 17 0 0 0 0 0 0 0 1 436 23 40 0 0 0 0 0 0 0 0 0 0 0 0 1 132 589 202 0 0 0 0 0 0 0 0 0 0 0 0 0 2 209 175 0 0 0 0 0 0 0 0 0 0 0 0 0 0 3 72 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 3 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 10 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 14 0 0 0 0 0 0 0 0 0 0 30 0 0 0 0 0 0 0 0 0 0 9 0 0 1 142 2 0 0 0 0 2 0 0 0 0 0 1 0 0 169 5 0 0 0 0 0 0 0 0 0 0 0 0 0 43 ];
rx_indices_histogram = [ 0 0 758 0 0 9 528 26 318 454 0 428 0 217 1271 168 52 5 88 92 0 0 0 0 3 2 0 0 0 0 0 0 0 0 0 0 252 148 0 84 1 177 0 1 7 3806 0 3 944 1029 0 323 ];
```

3. Prune the codebooks based on the statistics (histograms):

At D:\github\5gm-lidar\mimo-matlab, run
```
upa_codebook_prune_unused
```
to generate 
```
Wrote tx_upa_codebook_16x16_N832_valid.mat with 109 codewords
Wrote rx_upa_codebook_4x4_N52_valid.mat with 28 codewords
prob of most popular Tx index=0.070395
prob of most popular Rx index=0.34
```
Now edit D:\github\5gm-data\getBestBeamsFromChannelRays.py to indicate the new '_valid' (add as suffix) codebooks (add tx and rx as prefixs)
```
txCodebookInputFileName = 'D:/github/5gm-lidar/matlab/tx_upa_codebook_16x16_N832_valid.mat'
rxCodebookInputFileName = 'D:/github/5gm-lidar/matlab/rx_upa_codebook_4x4_N52_valid.mat'
```

4. Run createBeamsOutputsAsNpz.py again

Run createBeamsOutputsAsNpz.py again to create final npz’s with smaller codebooks (files in the output folder will be overwritten)
```
D:\gits\lasse\software\5gm-lidar>python createBeamsOutputsAsNpz.py
```
Note the new histograms do not have zeros:
```
total numOfInvalidChannels =  9666
total numOfValidChannels =  11194
Sum =  20860
total numNLOS =  4712
total numLOS =  6482
Sum =  11194
tx_indices_histogram = [ 82 403 336 190 1 591 2 396 788 1 2 283 37 67 455 2 1 3 168 55 95 228 3 1 12 83 98 10 5 126 84 7 70 91 63 12 7 78 87 16 15 8 3 16 25 19 93 17 7 9 4 10 3 1 230 3 216 55 4 201 2 381 1 13 21 132 5 1 218 1 56 13 157 7 612 79 18 13 369 6 777 17 1 436 23 40 1 132 589 202 2 209 175 3 72 1 3 10 14 30 9 1 142 2 2 1 169 5 43 ];
rx_indices_histogram = [ 758 9 528 26 318 454 428 217 1271 168 52 5 88 92 3 2 252 148 84 1 177 1 7 3806 3 944 1029 323 ];
```
The files output_e_XXX.npz and outputs_positions_e_XXX.hdf5 have the results for each beam pair.

5. Generate ML outputs

Edit createBeamsOutputsAsNpz.py. You will need to inform the codebook sizes (version "valids").
```
cd D:\gits\lasse\software\5gm-lidar
python createBeamsOutputsAsNpz.py
```
This saves file beams_output.npz that is the output of the neural nets.


6)Beam-selection using LIDAR data
============
1. Generating ML inputs:

Assuming that we have the output from previous step, we get the LIDAR data as inputs for machine learning from the created folder in the 4) process,
using the createInputFromLIDAR.py

```
5gm-lidar>python createInputFromLIDAR.py obstaclematricesfolder beams_input.npz
```

This saves file beams_input.npz that is the input of the neural nets.

2. Running the Classifier:

Make sure you have the output npz file and the input file matching (obtained from the same list of examples). Then run:

```
cd D:\gits\lasse\software\5gm-lidar\
python classifierTopKBeams.py
```


7)Beam-selection using the positions of the vehicles
============
TBD
