#pragma once
#include "../../ORUtils/FileIO.h"
#include "../../CPFSegmentation/config.h"

namespace SCFUSION {
    namespace IO {
        class CPFConfigIO : public FileIO {
            APC::Config *config;
        public:
            explicit CPFConfigIO(APC::Config *config):config(config) {}

            void saveBinary(const std::string & path){
                std::fstream file(path, std::ios::out | std::ios::binary);
                saveBinary(&file);
                file.close();
            }

            void saveBinary(std::fstream *file){
                if (!file->is_open()) throw std::runtime_error("unable to open file.\n");
                file->write((char *) config, sizeof(APC::Config));
            }

            void loadBinary(const std::string &path){
                std::fstream file(path, std::ios::in | std::ios::binary);
                file.read((char *) config, sizeof(APC::Config));
            }

            void saveParams(const std::string &path){
                std::fstream file(path, std::ios::out);
                save(&file);
                file.close();
            }
            void loadParams(const std::string &path){
                load(path);
            }
//
            void save(std::fstream *file){
                if (!file->is_open()) throw std::runtime_error("unable to open file.\n");
                (*file) << "# Supervoxel Params\n";
                saveParam(packageName(&config->voxel_resolution, 1), (*file));
                saveParam(packageName(&config->seed_resolution, 1), (*file));
                saveParam(packageName(&config->color_importance, 1), (*file));
                saveParam(packageName(&config->spatial_importance, 1), (*file));
                saveParam(packageName(&config->normal_importance, 1), (*file));
                saveParam(packageName(&config->use_single_cam_transform, 1), (*file));
                saveParam(packageName(&config->use_supervoxel_refinement, 1), (*file));

                (*file) << "# Plane Params\n";
                saveParam(packageName(&config->use_random_sampling, 1), (*file));
                saveParam(packageName(&config->noise_threshold, 1), (*file));
                saveParam(packageName(&config->smooth_cost, 1), (*file));
                saveParam(packageName(&config->min_inliers_per_plane, 1), (*file));
                saveParam(packageName(&config->min_plane_area, 1), (*file));
                saveParam(packageName(&config->label_cost, 1), (*file));
                saveParam(packageName(&config->max_num_iterations, 1), (*file));
                saveParam(packageName(&config->max_curvature, 1), (*file));
                saveParam(packageName(&config->gc_scale, 1), (*file));
            }

            void load(const std::string &path){
                auto inputData = readFileToMap(path);
                loadParam(packageName(&config->voxel_resolution, 1), inputData);
                loadParam(packageName(&config->seed_resolution, 1), inputData);
                loadParam(packageName(&config->color_importance, 1), inputData);
                loadParam(packageName(&config->spatial_importance, 1), inputData);
                loadParam(packageName(&config->normal_importance, 1), inputData);
                loadParam(packageName(&config->use_single_cam_transform, 1), inputData);
                loadParam(packageName(&config->use_supervoxel_refinement, 1), inputData);

                loadParam(packageName(&config->use_random_sampling, 1), inputData);
                loadParam(packageName(&config->noise_threshold, 1), inputData);
                loadParam(packageName(&config->smooth_cost, 1), inputData);
                loadParam(packageName(&config->min_inliers_per_plane, 1), inputData);
                loadParam(packageName(&config->min_plane_area, 1), inputData);
                loadParam(packageName(&config->label_cost, 1), inputData);
                loadParam(packageName(&config->max_num_iterations, 1), inputData);
                loadParam(packageName(&config->max_curvature, 1), inputData);
                loadParam(packageName(&config->gc_scale, 1), inputData);
            }
        };
    }
}