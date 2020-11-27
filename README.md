# SCFusion
![teaser](https://github.com/ShunChengWu/SCFusion_Network/blob/main/img/landscape_teaser.png)

**Authors**: [Shun-Cheng Wu][sc], [Keisuke Tateno][keisu], [Nassir Navab][nassir] and [Federico Tombari][fede]

[sc]:http://campar.in.tum.de/Main/ShunChengWu
[keisu]:http://campar.in.tum.de/Main/KeisukeTateno
[nassir]:http://campar.in.tum.de/Main/NassirNavabCv
[fede]:http://campar.in.tum.de/Main/FedericoTombari

This is an implementation of our [SCFusion](https://arxiv.org/abs/2010.13662) on [InfiniTAM](https://github.com/victorprad/InfiniTAM) for open source purpose. 

## Dependencies
```
apt-get update
```
#####CMAKE
```
apt install python3-pip
pip3 install cmake
```
#####Eigen3
```
git clone https://github.com/eigenteam/eigen-git-mirror.git
cd eigen-git-mirror; mkdir build;cd build;cmake ..; make install
```
##### PCL
`sudo apt install libpcl-dev`

##### OpenCV
```
git clone https://github.com/opencv/opencv.git
cd opencv; mkdir build; cd build;
cmake ..; make; make install
```
##### glfw3 (if with GUI)
```
apt-get install libglfw3 libglfw3-dev
```
##### Assimp (if with data generator)
```
apt-get install libassimp-dev assimp-utils
```
##### TooN (if with data generator)
```
git clone https://github.com/edrosten/TooN.git; cd TooN;
./configure && make && make install
```
##### CUDA
Please follow the instruction on NVIDIA offical site.
assimp (for data generation).

##### libTorch
libTorch : You can download the pre-built version from Pytorch [website](https://pytorch.org/get-started/locally/)
or build it from [source](https://github.com/pytorch/pytorch#from-source)


## Using docker
If you have nvidia-docker2:
```
docker pull gn02077706/ubuntu18.04_cuda_dev:torch
```
else 
```
# only if you haven't installed docker yet
curl https://get.docker.com | sh
sudo systemctl start docker
sudo systemctl enable docker

# set up container toolkits
distribution=$(. /etc/os-release;echo $ID$VERSION_ID) \
 && curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add - \
 && curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | sudo tee /etc/apt/sources.list.d/nvidia-docker.list

# install nvidia-docker2
sudo apt-get update
sudo apt-get install -y nvidia-docker2

# restart docker
sudo systemctl restart docker
```
The docker image has all the dependencies our method needs. 





## Build
On your system or withint docker:
```
git clone https://github.com/ShunChengWu/SCFusion.git
cd SCFusion; mkdir build; cd build; 

#`-DBUILD_DATA_GENERATOR=True` to build data generator. 
#`-DWITH_PYTORCH=True -DPYTORCH_PATH=/path/to/libtorch/` to enable pytorch support
#`-DWITH_GUI=ON` to build with GUI
cmake ..;
make
```


## Scene reconstruction
Our system takes a configuration file. You can find an example file under Configurations folder.
It also takes some command line arguments, you can find them by pass `--h` or in */SLAMWrapper/include/SLAMTools/Parser.hpp*.

The trained model can be downloaded from [link](http://campar.in.tum.de/files/scfusion/SI_ScanNet_0614.pt).
```
mkdir Models;
cd Models;
wget http://campar.in.tum.de/files/scfusion/SI_ScanNet_0614.pt
cd ../
``` 
Change `libSettings->scParams.pth_to_pb` in a config file to the this model. 

To reconstruct
```
# useGTPose 0: no, 1: assist, 2: yes
./build/App/SCFusion/exe_scfusion_OFusionRGB1Label ./Configurations/Config_ScanNet_CRF.txt --useGTPose 2 --useSC 1 --pthOut /pth/to/output/folder/
# or with GUI (left button: translation; right button: rotation; scrolling: zoom. )
./build/App/SCFusion/exe_scfusion_gui_OFusionRGB1Label ./Configurations/Config_ScanNet_CRF.txt --useGTPose 2 --useSC 1 --pthOut /pth/to/output/folder/
```

Get mesh
```
./exe_Map2Mesh_OFusionRGB1Label --pth_in /pth/to/output/folder/ --pth_out /pth/to/output_mesh/folder/
```


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