import subprocess, os, sys, time
import multiprocessing as mp
import copy
debug=0
exe='../cmake-build-release/App/TrainingDataGenerator/exe_TrainingDataGenerator_ScanNetScan2CAD'
config='../Configurations/Config_ScanNet.txt'
ITSDF=0
fromGT=1
threshold_o=0.98
threshold_h=0.7
threshold_occupancy_rc=0.001
minLabelNum=2
labelNum=12
thread=1
dryrun=0
loadLocation="-1"
checkState=1
outputVoxelDistance=0
volumeDims = [64, 64, 64]
makeInitValueInf=0
voxelSize = 0.05
mu = 0.150
ofuNorm=1
gtFromRC=0
pth_location=""
sampleNum=100
skipFrame = 200
renderFilterThreshold=-1

pth_ply='/media/sc/space1/dataset/scannet/scans'
pth_scan='/media/sc/SSD1TB/scannet_poses'
pth_shapenet='/media/sc/space1/dataset/ShapeNetCore.v2/'
pth_annotations='/media/sc/space1/dataset/Scan2CAD/Routines/Script/full_annotations.json'

baseFolder = '/media/sc/BackupDesk/TrainingDataScanNet_0614/047_200/'
baseFolder = '/media/sc/BackupDesk/tmp/'

train_file='/media/sc/space1/dataset/scannet/Tasks/Benchmark/scannetv2_train.txt'
test_file='/media/sc/space1/dataset/scannet/Tasks/Benchmark/scannetv2_val.txt'


#################################################
height=volumeDims[1]
output_log_path= baseFolder
output_training_path= baseFolder + 'train/'
output_test_path= baseFolder + 'test/'

if pth_location != "":
    pth_location_train = pth_location + 'train/'
    pth_location_test = pth_location + 'test/'
else:
    pth_location_train = ''
    pth_location_test = ''

def createFolder(directory):
    try:
        if not os.path.exists(directory):
            os.makedirs(directory)
    except OSError:
        print ('Error: Creating directory. ' +  directory)

def process(idx, targetScan, folder, pth_locate_file, pathOut):
    startTime = time.time()
    try:
        print('Processing file', targetScan)
        output = subprocess.check_output([exe, config, 
                     '--pth_ply', pth_ply,
                     '--pth_scan', pth_scan,
                     '--pth_shapenet',pth_shapenet,
                     '--pth_annotations',pth_annotations,
                     '--pthOut', pathOut,
                     '--target_scene',str(idx),
                     '--sampleNum', str(sampleNum),
                     '--height', str(height),
                     '--fromGT', str(fromGT),
                     '--ITSDF', str(ITSDF),
                     '--minLabelNum',str(minLabelNum),
                     '--labelNum',str(labelNum),
                     '--threshold_o', str(threshold_o),
                     '--threshold_h', str(threshold_h),
                     '--threshold_occupancy_rc',str(threshold_occupancy_rc),
                     '--dryrun', str(dryrun),
                     '--pth_location', pth_locate_file,
                     '--checkState', str(checkState),
                     '--outputVoxelDistance', str(outputVoxelDistance),
                     '--makeInitValueInf', str(makeInitValueInf),
                     '--voxelSize', str(voxelSize),
                     '--mu', str(mu),
                     '--volumeDims', str(volumeDims[0]), str(volumeDims[1]),str(volumeDims[2]),
                     '--ofuNorm',str(ofuNorm),
                     '--gtFromRC',str(gtFromRC),
                     '--skipFrame',str(skipFrame),
                     '--renderFilterThreshold',str(renderFilterThreshold),
                     #'--gui',
                     #'--verbose',
                     ],
            stderr=subprocess.STDOUT)
        sys.stdout.write(output.decode('utf-8'))
    except subprocess.CalledProcessError as e:
        print('[Catched Error]', "command '{}' return with error (code {}): {}".format(e.cmd, e.returncode, e.output.decode('utf-8'))) # omit errors
    endTime = time.time()
    print('Processing file', targetScan, 'finished')
    return [targetScan, endTime-startTime]
    
def writeLog(times,path):
    with open(path, "w+") as f:
        f.write('params:\n')
        print("config {}".format(config), file=f)
        f.write('pth_scan {}\n'.format(pth_scan))
        f.write('pth_shapenet {}\n'.format(pth_shapenet))
        f.write('pth_annotations {}\n'.format(pth_annotations))
        f.write('train_file {}\n'.format(train_file))
        f.write('test_file {}\n'.format(test_file))
        f.write('sampleNum {}\n'.format(sampleNum))
        f.write('skipFrame {}\n'.format(skipFrame))
        f.write('height {}\n'.format(height))
        f.write('threshold_o {}\n'.format(threshold_o))
        f.write('threshold_h {}\n'.format(threshold_h))
        f.write('threshold_occupancy_rc {}\n'.format(threshold_occupancy_rc))
        f.write('fromGT {}\n'.format(fromGT))
        f.write('ITSDF {}\n'.format(ITSDF))
        f.write('minLabelNum {}\n'.format(minLabelNum))
        f.write('labelNum {}\n'.format(labelNum))
        f.write('dryrun {}\n'.format(dryrun))
        f.write('loadLocation {}\n'.format(loadLocation))
        f.write('checkState {}\n'.format(checkState))
        f.write('outputVoxelDistance {}\n'.format(outputVoxelDistance))
        f.write('makeInitValueInf {}\n'.format(makeInitValueInf))
        f.write('voxelSize {}\n'.format(voxelSize))
        f.write('mu {}\n'.format(mu))
        f.write('volumeDims {} {} {}\n'.format(volumeDims[0], volumeDims[1],volumeDims[2]))
        f.write('ofuNorm {}\n'.format(ofuNorm))
        f.write('gtFromRC {}\n'.format(gtFromRC))
        f.write('renderFilterThreshold {}\n'.format(renderFilterThreshold))
        f.write('\n')
        f.write('Times\n')
        for a,b in times.items():
            f.write('{} {}\n'.format(a,b))
            
def loadMap(log_pth):
    totalTime={}
    try:
        with open(log_pth) as f:
            for line in f:
               try:
                   (key, val) = line.split()
               except:
                   continue
               totalTime[key] = val
    except IOError: 
       print('No previous file found.{}'.format(log_pth))
    return totalTime

def getFolderList(path):
    pbPaths=[]
    if os.path.isfile(path):
        pbPaths = [path]
    else:
#        pbPaths = [os.path.join(path,p) for p in sorted(os.listdir(path))]
        pbPaths = sorted(os.listdir(path))
    return pbPaths

def ReadFileToList(path):
    with open(path) as f:
      content = f.readlines()
    content = [x.strip() for x in content]
    return content

import random
def launchProcess(indices, folders, baseFolder, output_path, pth_location_file, pool, totalTime):
    results=[]
    for idx in indices:
#        idx = random.choice(indices)
#        print('random idx:',idx, folders[idx])
        folder = os.path.join(baseFolder, folders[idx])
        #print(folder)
        if os.path.isdir(folder): 
            pbPath= folder
            pbName=os.path.splitext(os.path.basename(folder))[0]
            
                
            if pbPath in totalTime:
                print('Skip file', pbPath)
#            else:
#                print('Processing file', pbPath)
                
            if debug > 0:
                results = process(idx,pbName, baseFolder, pth_location_file, output_path)
            else:
                results.append(
                    pool.apply_async(process, (idx, pbName, baseFolder, pth_location_file, output_path))
                )
            if debug > 0:
                break
        # break
    return results
def GenTrainingSet(totalTime):
    pool = mp.Pool(thread)
    pool.daemon = True
    
    all_scans=getFolderList(pth_scan)
    all_scans.sort()

    # Train
    results_train=None
    folders=ReadFileToList(train_file)
    folders.sort()
    indices = [all_scans.index(i) for i in folders]

#    import sys
#    sys.exit()

    results_train = launchProcess(indices, all_scans, pth_scan, output_training_path, pth_location_train, pool, totalTime)
    
    # Test
    results_test=None
    folders=ReadFileToList(test_file)
    folders.sort()
    indices = [all_scans.index(i) for i in folders]
    results_test = launchProcess(indices, all_scans, pth_scan, output_test_path, pth_location_test, pool, totalTime)
    
    pool.close()
    pool.join()
    if results_train is not None:
      if debug == 0:
        results = [r.get() for r in results_train]
      else:
        results = [results_train]
      [print(r) for r in results]
      for r in results:
        totalTime[r[0]] = r[1]
    if results_test is not None:
      if debug == 0:
        results = [r.get() for r in results_test]
      else:
        results = [results_test]
      [print(r) for r in results]
      for r in results:
        totalTime[r[0]] = r[1]
            
if __name__ == '__main__':
    createFolder(output_training_path)
    createFolder(output_test_path)
    createFolder(output_log_path)
    log_pth = os.path.join(output_log_path,'log.txt')
    totalTime = loadMap(log_pth)
    
    GenTrainingSet(totalTime)

    writeLog(totalTime,log_pth)
