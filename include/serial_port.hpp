#ifndef SERIAL_PORT_HPP_
#define SERIAL_PORT_HPP_

#include <stdio.h>    // Standard input/output definitions
#include <stdint.h>   // Standart types definitions
#include <string.h>   // String function definitions
#include <unistd.h>   // UNIX standard function definitions
#include <fcntl.h>    // File control definitions
#include <errno.h>    // Error number definitions
#include <linux/serial.h>
#include <termios.h>  // POSIX terminal control definitions
#include <sys/ioctl.h>
#include <string>

/* #include <boost/thread.hpp> */
/* #include <boost/function.hpp> */

class SerialPort {
public:
  SerialPort();
  virtual ~SerialPort();

  bool virtual_ = false;

  bool connect(const std::string port, const int baudrate, const bool virtual_comm);
  void disconnect();

  bool sendChar(const char c);
  bool sendCharArray(uint8_t* buffer, int len);

  void setBlocking(int fd, int should_block);

  bool checkConnected();

  /**
   * @param arr buffer array to long incoming data
   * @param arr_max_size length of the buffer array
   * 
   * @return 0U in case of out of file or error occurred,
   * @return number of read bytes otherwise
   */
  uint32_t readSerial(uint8_t* arr, uint32_t arr_max_size);

  int      serial_port_fd_;
  uint8_t  input_buffer[1024];
  uint16_t input_it = 0;
};

#endif  // SERIAL_PORT_HPP_