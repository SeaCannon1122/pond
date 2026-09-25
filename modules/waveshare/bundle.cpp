#include <pond/pond.hpp>

EXTERN_POND_MODULE(DDSM115Driver);
EXTERN_POND_MODULE(ServoDriver);

POND_BUNDLE_DECLARE(
    "Modules for waveshare hardware", 
    POND_MODULE(DDSM115Driver),
    POND_MODULE(ServoDriver)
)