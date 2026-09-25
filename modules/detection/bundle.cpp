#include <pond/pond.hpp>

EXTERN_POND_MODULE(QRCodeDetector);
EXTERN_POND_MODULE(LandoltCDetector);

POND_BUNDLE_DECLARE(
    "Detection stuff",
    POND_MODULE(QRCodeDetector),
    POND_MODULE(LandoltCDetector)
)