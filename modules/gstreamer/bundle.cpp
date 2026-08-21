#define POND_MODULE_CPP_MAKE_IMPLEMENTATION
#include <pond/pond.hpp>

EXTERN_POND_MODULE(RTPServer);

POND_BUNDLE_DECLARE(
    "Gstreamer modules", 
    1,
    POND_MODULE(RTPServer),
)