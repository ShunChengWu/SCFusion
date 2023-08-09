#pragma once

#include <fstream>
#include <map>
#include <string>
#include <vector>
#include <numeric>
#include <CxxTools/PathTool.hpp>
#include "../../../MainLib/Objects/Scene/ITMVoxelTypes.h"
#include "../../../MainLib/ITMLibDefines.h"
#include "../../../MainLib/Utils/ITMLibSettings.h"
#include "../../../MainLib/Utils/LibSettingsIO.h"
#include <ORUtils/Logging.h>
#include <ORUtils/LogUtil.h>

namespace SCFUSION {
    namespace EVALUATION {
        class Logging {
        public:
            Logging()=default;
            ~Logging()= default;
            // Suppress the default copy constructor and assignment operator
            Logging(const Logging&);
            Logging& operator=(const Logging&);


            static std::string GenerateOutputPathAndName(std::string pth_out) {
                return pth_out;
            }

            static bool saveRuntimeParams(const std::string &pth_out, ITMLib::ITMLibSettings *settings, const std::string &filename = "params.txt"){
                checkInputFolder(pth_out);
                const std::string outputFileName = tools::PathTool::CheckEnd(pth_out) + filename;
                std::fstream logFile_(outputFileName, std::ios::out);
                if(!logFile_.is_open()) {
                    SCLOG(ERROR) <<"Unable to open file at " << outputFileName << "\n";
                    return false;
                }

                SCFUSION::IO::LibSettingsIO libSettingsIo(settings);
                libSettingsIo.saveParams(&logFile_);
                printf("Runtime params saved to %s\n", outputFileName.c_str());
                return true;
            }

            static void saveTimeInfo(std::map<std::string, std::vector<double>> &times_){
                /// Save Time Information
                for (const std::pair<std::string, double> &time : getWatch.getTimings()) {
                    if(!getWatch.updated(time.first)) continue;
                    getWatch.getUpdateStats()[time.first] = false;

                    if(times_.find(time.first) == times_.end()) {
                        times_[time.first].reserve(1e5); // assume will have this amount of images
                        times_[time.first].push_back(time.second);
                    } else {
                        times_[time.first].push_back(time.second);
                    }
                }
            }

            static bool printResultToFile(const std::map<std::string, std::vector<double>> & times_, const std::string &pth_out, const std::string &filename = "times.csv") {
                checkInputFolder(pth_out);
                const std::string outputFileName = tools::PathTool().CheckEnd(pth_out) + filename;
                std::fstream logFile_(outputFileName, std::ios::out);
                if(!logFile_.is_open()) {
                    SCLOG(ERROR) <<"Unable to open file at " << outputFileName << "\n";
                    return false;
                }

                if(!logFile_.is_open()) return false;
                logFile_ << "Name;Mean;Variance;Standard Deviation;Number of Measurements" << ";\n";
                double mean, var, stdvar;
                for (auto &time : times_) {
                    mean = var = stdvar = 0;
                    if (time.second.size() == 1) {
                        mean = time.second[0];
                        var = stdvar = 0;
                    } else {
                        mean = std::accumulate(time.second.begin(), time.second.end(), 0.f) / time.second.size();
                        for(size_t i=0;i<time.second.size();++i){
                            auto d = mean - time.second[i];
                            var += d*d;
                        }
                        var /= time.second.size();
                        stdvar = std::sqrt(var);
                    }

                    logFile_ << time.first << ";" << std::to_string(mean) << ";" << std::to_string(var) << ";" << std::to_string(stdvar)
                       << ";" << std::to_string(time.second.size()) << "\n";
                }

//                for(auto &time:times_){
//                    if(time.first == "[SLAMWrapper][processOnce]3.slam_->ProcessFrame" ||
//                            time.first == "[SLAM][ProcessFrame]4.Mapping") {
//                        logFile_ << time.first<<"\n";
//                        for(size_t i=0;i<time.second.size();++i){
//                            logFile_ << i << ";" <<time.second[i] << "\n";
//                        }
//                    }
//
//                }

                logFile_.close();
                printf("Runtime saved to %s\n", outputFileName.c_str());
                return true;
            }

        private:
            static bool checkInputFolder(const std::string &pth_out){
                tools::PathTool pathTool;
                if(!pathTool.isFolder(pth_out)) {
                    SCLOG(ERROR) << "expect a folder. Given was not.\n";
                    return false;
                }
                pathTool.check_and_create_folder(pth_out);
                return true;
            }
        };
    }
}