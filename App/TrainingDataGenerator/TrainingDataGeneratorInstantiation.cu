#include "TrainingDataGenerator.cu"

template class SCFUSION::TrainingDataGenerator<TSDF_Voxel_f_2label,OFu_Voxel_f_2label>;
template class SCFUSION::TrainingDataGenerator<TSDF_Voxel_f_2label,TSDF_Voxel_f_2label>;
template class SCFUSION::TrainingDataGenerator<OFu_Voxel_f_2label,OFu_Voxel_f_2label>;


//template class SCFUSION::TrainingDataGenerator<TSDF_Voxel_f_label,OFu_Voxel_f_1label>;
//template class SCFUSION::TrainingDataGenerator<TSDF_Voxel_f_label,TSDF_Voxel_f_label>;
//template class SCFUSION::TrainingDataGenerator<OFu_Voxel_f_1label,OFu_Voxel_f_1label>;
