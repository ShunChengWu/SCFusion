#!/bin/bash
# This script rotate ScanNet *_vh_clean_2.labels.ply to align with the world coordinate system we used in our method

# DEFINE PATHS #
exe_rotate_mesh='../build/App/rotation/exe_rotate_mesh' #'/path/to/exe_rotate_mesh'
scannetfolder='/media/sc/space1/dataset/scannet/scans' # /path/to/scannet/scans

####
angle_x=-0.5
angle_y=-0.5
angle_z=0

echo "Processing..."
for d in $scannetfolder/*/ ; do
  # check whether *.sens file exist
  for name in $d*vh_clean_2.labels.ply ; do
    if [ -f "$name" ]; then
      filename=$(basename -- "$name")
      extension="${filename##*.}"
      filename="${filename%.*}"
#      echo "name: " $name      
#      echo "filename: " $filename      
#      echo "extension: " $extension
      #echo "[BASH] --------Run Data Poseprocessing Pipieline--------"
      # Check ground truth binary file exist. If not, generate
      outFliped=$d/$filename"_flip2."$extension
      $exe_rotate_mesh --pth_in $name --angle_x $angle_x --angle_y $angle_y --angle_z $angle_z
      exit
    fi
  done
done
echo "Done."


