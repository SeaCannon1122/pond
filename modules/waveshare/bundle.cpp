#define POND_MODULE_CPP_MAKE_IMPLEMENTATION
#include <pond/pond.hpp>

EXTERN_POND_MODULE(DDSM115Driver);

POND_BUNDLE_DECLARE(
    "Modules for waveshare hardware", 
    1,
    POND_MODULE(DDSM115Driver),
)