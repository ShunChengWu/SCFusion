#pragma once

#include "../../ORUtils/FileIO.h"
#include "ITMLibSettings.h"
#include <cstdlib>
#include <map>
#include <vector>
#include <fstream>
#include <utility>
//#include "../../Utilities/include/Utilities/Exception.hpp"

namespace SCFUSION{
    namespace IO {
        class LibSettingsIO : public FileIO {
            ITMLib::ITMLibSettings *libSettings;
        public:
            explicit LibSettingsIO(ITMLib::ITMLibSettings *libSettings_) : libSettings(libSettings_) {}

            void saveParams(const std::string &path);

            void saveParams(std::fstream *file);

            void loadParams(const std::string &path);
        };

        template <> void FileIO::assignValue(SCFUSION::Policy::Integrate *policy, const std::vector<std::string> &map);

        template <> void FileIO::assignValue(SCFUSION::Policy::FuseTwo *policy, const std::vector<std::string> &map);

        template <> void FileIO::assignValue(ITMLib::ITMLibSettings::DeviceType *type, const std::vector<std::string> &map);

        template <> void FileIO::assignValue(ITMLib::ITMLibSettings::FailureMode *mode, const std::vector<std::string> &map);

        template <> void FileIO::assignValue(ITMLib::ITMLibSettings::SwappingMode *mode, const std::vector<std::string> &map);

        template <> void FileIO::assignValue(ITMLib::ITMLibSettings::LibMode *mode, const std::vector<std::string> &map);

        template <> void FileIO::assignValue(SCFUSION::SceneCompletionMethod *method, const std::vector<std::string> &map);
    }
}
