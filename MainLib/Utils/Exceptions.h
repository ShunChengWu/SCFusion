#pragma once
#include <cstdlib>


namespace SCFUSION {
    class ERROR_NotImplemented : public std::logic_error {
    public:
        explicit ERROR_NotImplemented(const std::string &functionName = "") : std::logic_error(functionName  + " Function not yet implemented") {};
    };
}
