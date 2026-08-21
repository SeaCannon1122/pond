#include <string>
#include <stdbool.h>
#include <stdint.h>

#define M_PI 3.14159265358979323846

enum DriveMode
{
    DRIVE_MODE_CURRENT = 1,
    DRIVE_MODE_VELOCITY = 2,
    DRIVE_MODE_POSITION = 3
};

class DDSM115CMD
{
public:
    bool connect(const std::string& port);
    void disconnect();
    const char* get_error();

    bool drive(uint8_t id, double velocity, uint8_t act, uint8_t brake);
    bool set_id(uint8_t id);
    bool set_mode(uint8_t id, int mode);
    bool drive_feedback(uint8_t* id, uint8_t* mode, double* current, double* velocity, double* position, uint8_t* error_code);
    int read_bytes(uint8_t* buf, size_t buf_size);

private:
    void set_error(char* str, ...);

    char m_Error[64];
    int m_SerialFD;
};