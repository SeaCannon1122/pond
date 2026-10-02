#include <pond/pond.hpp>

EXTERN_POND_MODULE(H264UDPStreamer);

POND_BUNDLE_DECLARE(
    "video streaming",
    POND_MODULE(H264UDPStreamer),
)