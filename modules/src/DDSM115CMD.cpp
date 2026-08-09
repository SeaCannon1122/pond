#include "DDSM115CMD.h"

#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <stdarg.h>
#include <cmath>

#include <chrono>
#include <thread>

const char* DDSM115CMD::get_error() { return m_Error; }

void DDSM115CMD::set_error(char* str, ...)
{
  va_list args;
  va_start(args, str);

  vsnprintf(m_Error, sizeof(m_Error), str, args);

  va_end(args);
}

bool DDSM115CMD::connect(const std::string& port)
{
  if ((m_SerialFD = open(port.c_str(), O_RDWR | O_NOCTTY)) < 0) {
    set_error("Error opening serial port");
    return false;
  }

  struct termios tty;
  if (tcgetattr(m_SerialFD, &tty) != 0)
  {
    set_error("Error from tcgetattr");

    close(m_SerialFD);
    return false;
  }

  cfsetospeed(&tty, B115200);
  cfsetispeed(&tty, B115200);

  tty.c_cflag &= ~PARENB; // No parity
  tty.c_cflag &= ~CSTOPB; // 1 stop bit
  tty.c_cflag &= ~CSIZE; // Clear byte size bits
  tty.c_cflag |= CS8; // 8 bits per byte
  tty.c_cflag &= ~CRTSCTS; // Disable CTS/RTS
  tty.c_lflag = 0; // Make tty raw
  tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
  tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes
  tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes
  tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
  tty.c_cc[VTIME] = 0;
  tty.c_cc[VMIN] = 0;

  if (tcsetattr(m_SerialFD, TCSANOW, &tty) != 0)
  {
    set_error("Error from tcsetattr");

    close(m_SerialFD);
    return false;
  }

  return true;
}

void DDSM115CMD::disconnect()
{
  tcdrain(m_SerialFD);
  close(m_SerialFD);
}

uint8_t maximCrc8(uint8_t* data, const unsigned int size)
{
  uint8_t crc = 0;
  for (unsigned int i = 0; i < size; ++i)
  {
    uint8_t inbyte = data[i];
    for (unsigned char j = 0; j < 8; ++j)
    {
      uint8_t mix = (crc ^ inbyte) & 0x01;
      crc >>= 1;
      if (mix)
        crc ^= 0x8C;
      inbyte >>= 1;
    }
  }
  return crc;
}

bool DDSM115CMD::set_id(uint8_t id)
{
  uint8_t cmd[] = 
  {
    0xAA,
    0x55,
    0x53,
    (uint8_t) id,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00 
  };

  ::write(m_SerialFD, cmd, sizeof(cmd));

  std::this_thread::sleep_for(std::chrono::milliseconds(5));

  tcdrain(m_SerialFD);

  return true;
}

bool DDSM115CMD::set_mode(uint8_t id, int mode)
{
  uint8_t cmd[] = 
  {
    (uint8_t) id,
    0xA0,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    (uint8_t) mode 
  };

  ::write(m_SerialFD, cmd, sizeof(cmd));

  std::this_thread::sleep_for(std::chrono::milliseconds(5));

  tcdrain(m_SerialFD);

  return true;
}

bool DDSM115CMD::drive(uint8_t id, double velocity, uint8_t act, uint8_t brake)
{
  int16_t rpm = (int16_t)(velocity / (2.0 * M_PI) * 60.0);

  uint8_t cmd[] = 
  {
    (uint8_t) id,
    100,
    (uint8_t)((rpm >> 8) & 0xFF),
    (uint8_t)(rpm & 0xFF),
    0x00,
    0x00,
    act,
    brake,
    0x00,
    0x00 
  };

  cmd[9] = maximCrc8(cmd, 9);

  ::write(m_SerialFD, cmd, sizeof(cmd));

  std::this_thread::sleep_for(std::chrono::milliseconds(5));

  tcdrain(m_SerialFD);

  return true;
}

bool DDSM115CMD::drive_feedback(uint8_t* id, uint8_t* mode, double* position, double* velocity, double* current, uint8_t* error_code)
{
  uint8_t response[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
  int total_num_bytes = 0;
  int num_bytes = 0;
  for (int j = 0; j < sizeof(response); j++) {
    num_bytes = ::read(m_SerialFD, &response[j], 1);
    if (num_bytes <= 0) {
      break;
    }
    total_num_bytes += num_bytes;
  }

  if (num_bytes < 0)
  {
    set_error("Error reading DDSM115 response");
    return false;
  }

  else if (total_num_bytes < 10)
  {
    set_error("Error reading DDSM115 response, only received %d bytes " 
      "%02x %02x %02x %02x %02x %02x %02x %02x %02x %02x", total_num_bytes,
      response[0], response[1], response[2], response[3], response[4], response[5], response[6], response[7], response[8], response[9]
    );
    return false;
  }
  else if (response[9] != maximCrc8(response, 9))
  {
    set_error("CRC error in response");
    return false;
  }

  int16_t drive_current = (response[2] << 8) + response[3];
  int16_t drive_velocity = (response[4] << 8) + response[5];
  uint16_t drive_position = (response[6] << 8) + response[7];

  *id = response[0];
  *mode = response[1];
  *current = (double)drive_current / 32768.0*8000.0;
  *velocity = (double)drive_velocity / 60.0 * 2.0 * M_PI;
  *position = (double)drive_position / 32768.0 * 2.0 * M_PI;
  *error_code = response[8];

  return true;
}

int DDSM115CMD::read_bytes(uint8_t* buf, size_t buf_size)
{
  return ::read(m_SerialFD, buf, buf_size);
}

