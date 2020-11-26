#pragma once

#include "../../ORUtils/FileIO.h"
//#include <Engines/Segmentation/Interface/InSegLabeling.h>
#include <Engines/Segmentation/Interface/InSegParams.h>
//#include <toml.hpp>

namespace SCFUSION {
    namespace IO {
        class InSegSettingsIO : public FileIO {
            InSeg::InSegParams *inseg_;
        public:
            explicit InSegSettingsIO(InSeg::InSegParams *inseg):inseg_(inseg) {}

            void saveBinary(const std::string & path){
                std::fstream file(path, std::ios::out | std::ios::binary);
                saveBinary(&file);
                file.close();
            }

            void saveBinary(std::fstream *file){
                if (!file->is_open()) throw std::runtime_error("unable to open file.\n");
                file->write((char *) inseg_, sizeof(InSeg::InSegParams));
            }

            void loadBinary(const std::string &path){
                std::fstream file(path, std::ios::in | std::ios::binary);
                file.read((char *) inseg_, sizeof(InSeg::InSegParams));
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
                saveParam(packageName(&inseg_->bUseEdgeGrowing, 1), (*file));
                saveParam(packageName(&inseg_->bUseEnahnceEdge, 1), (*file));
                saveParam(packageName(&inseg_->minSegmentSize, 1), (*file));
                saveParam(packageName(&inseg_->depthUncertaintyCoef, 1), (*file));
                saveParam(packageName(&inseg_->depthEdgeThresh, 1), (*file));

                saveParam(packageName(&inseg_->initLabelOnly, 1), (*file));
                saveParam(packageName(&inseg_->matchingLabelRatioThresh, 1), (*file));
                saveParam(packageName(&inseg_->newlabelSizeRatioThresh, 1), (*file));
                saveParam(packageName(&inseg_->connectLabelSizeThresh, 1), (*file));
                saveParam(packageName(&inseg_->labelMergingConfidenceThresh, 1), (*file));
                saveParam(packageName(&inseg_->candRegionRatio, 1), (*file));

                saveParam(packageName(&inseg_->bUseCannyEdge_, 1), (*file));
                saveParam(packageName(&inseg_->lowB, 1), (*file));
                saveParam(packageName(&inseg_->highB, 1), (*file));
                saveParam(packageName(&inseg_->h, 1), (*file));
                saveParam(packageName(&inseg_->hColor, 1), (*file));
                saveParam(packageName(&inseg_->templateWindowSize, 1), (*file));
                saveParam(packageName(&inseg_->searchWindowSize, 1), (*file));
            }

            void load(const std::string &path){
                auto inputData = readFileToMap(path);
                loadParam(packageName(&inseg_->bUseEdgeGrowing, 1), inputData);
                loadParam(packageName(&inseg_->bUseEnahnceEdge, 1), inputData);
                loadParam(packageName(&inseg_->minSegmentSize, 1), inputData);
                loadParam(packageName(&inseg_->depthUncertaintyCoef, 1), inputData);
                loadParam(packageName(&inseg_->depthEdgeThresh, 1), inputData);

                loadParam(packageName(&inseg_->initLabelOnly, 1), inputData);
                loadParam(packageName(&inseg_->matchingLabelRatioThresh, 1), inputData);
                loadParam(packageName(&inseg_->newlabelSizeRatioThresh, 1), inputData);
                loadParam(packageName(&inseg_->connectLabelSizeThresh, 1), inputData);
                loadParam(packageName(&inseg_->labelMergingConfidenceThresh, 1), inputData);
                loadParam(packageName(&inseg_->candRegionRatio, 1), inputData);

                loadParam(packageName(&inseg_->bUseCannyEdge_, 1), inputData);
                loadParam(packageName(&inseg_->lowB, 1), inputData);
                loadParam(packageName(&inseg_->highB, 1), inputData);
                loadParam(packageName(&inseg_->h, 1), inputData);
                loadParam(packageName(&inseg_->hColor, 1), inputData);
                loadParam(packageName(&inseg_->templateWindowSize, 1), inputData);
                loadParam(packageName(&inseg_->searchWindowSize, 1), inputData);
            }
        };
    }
}