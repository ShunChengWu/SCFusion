# SCFusion
![teaser](https://github.com/ShunChengWu/SCFusion_Network/blob/main/img/landscape_teaser.png)

**Authors**: [Shun-Cheng Wu][sc], [Keisuke Tateno][keisu], [Nassir Navab][nassir] and [Federico Tombari][fede]

[sc]:http://campar.in.tum.de/Main/ShunChengWu
[keisu]:http://campar.in.tum.de/Main/KeisukeTateno
[nassir]:http://campar.in.tum.de/Main/NassirNavabCv
[fede]:http://campar.in.tum.de/Main/FedericoTombari

This is an implementation of our [SCFusion](https://arxiv.org/abs/2010.13662) on [InfiniTAM](https://github.com/victorprad/InfiniTAM) for open source purpose. 


## Dependencies
CMake, CUDA, OpenCV 4.x, Eigen3, PCL, assimp (for data generation).

libTorch : You can download the pre-built version from Pytorch [website](https://pytorch.org/get-started/locally/)
or build it from [source](https://github.com/pytorch/pytorch#from-source)

## Build
`-DBUILD_DATA_GENERATOR=True` to build data generator. 

`-DWITH_PYTORCH=True -DPYTORCH_PATH=/path/to/libtorch/` to enable pytorch support

`-DWITH_GUI=ON` to build with GUI
```
mkdir build; 
cd build; 
cmake -DWITH_PYTORCH=True -DPYTORCH_PATH=/path/to/libtorch/ ..; 
make
```

**Possible issue** 

In libTorch, it uses keyword signatures to link libraries while in FindCUDA.cmake it uses plain library linking. 
This may cause issues when building this project. You may need to change the target_link_libraries in FindCUDA.cmake to
signature-based linking.

## Scene reconstruction
You can use our trained model from [link](http://campar.in.tum.de/files/scfusion/SI_ScanNet_0614.pt). 
Change `libSettings->scParams.pth_to_pb` in a config file to the this model. 

To reconstruct
```
./build/App/SCFusion/exe_scfusion_OFusionRGB1Label ./Configurations/Config_ScanNet_CRF.txt --useGTPose 2 --useSC 1 --pthOut /pth/to/output/folder/
# or with GUI
./build/App/SCFusion/exe_scfusion_gui_OFusionRGB1Label ./Configurations/Config_ScanNet_CRF.txt --useGTPose 2 --useSC 1 --pthOut /pth/to/output/folder/
# useGTPose 0: no, 1: assist, 2: yes
```

Get mesh
```
./exe_Map2Mesh_OFusionRGB1Label --pth_in /pth/to/output/folder/ --pth_out /pth/to/output_mesh/folder/
```
Our system assumes floor-aligned reconstruction. We use a simple floor detection method on the first frame if no ground truth poses are given.

## CompleteScanNet Data Generation
Our dataset is built based on [ScanNet][scannet], [Scan2CAD][scan2cad] and [ShapeNetCore.v2][shapenet]. You will need
to download the dataset from them first in order to build CompleteScanNet.

You will need to download the `*_vh_clean_2.labels.ply` and `*.sens` files.

[scannet]:https://github.com/ScanNet/ScanNet
[scan2cad]:https://github.com/skanti/Scan2CAD
[shapenet]:https://www.shapenet.org/

#### Ground Truth Generation
**Flip Clouds**

Our system uses a different coordinate system than ScanNet. The input ply files must be flipped in order to correctly 
align it to our system.

It's also possible to use the original ScanNet coordinate system, but you will need to change 
some codes accordingly. We are working on that. 
```
# Flip mesh
cd scripts
bash RUN_FLIP_SCANNET_MESH.sh # Need to change the paths inside  

# Generate aligned poses
cd ../build/App/extractPoseFromScanNet;
./exe_ExtractPoseFromScanNet --pth_in /pth/to/scannet/scans/ --pth_out /pth/to/output/pose_folder/
```

##### Generate GT
replace `--fill 1` to `--fill 0` if you want to generate not filled ground truth. 
```
# Prepare ground truth
./exe_GroundtruthGenerator_ScanNetScan2CAD \
../../Configurations/Config_ScanNet.txt \
 --pth_scan /pth/to/scannet/scans/ \
 --pth_shapenet  /pth/to/shanetcore.v2/ \
 --pth_annotations /pth/to/Scan2CAD/Routines/Script/full_annotations.json \
 --gui 0 \
 --verbose 0 \
 --pthOut /path/to/output/folder/ \
 --fill 1 \
 --labelNum 12
```

#### Generate Training data
```
cd scripts
# Need to change paths
python3 GenerateTrainingData_ScanNetScan2CAD_skip200.py
```


## Network 
Please check the network repository: [SCFusion_Network](https://github.com/ShunChengWu/SCFusion_Network)