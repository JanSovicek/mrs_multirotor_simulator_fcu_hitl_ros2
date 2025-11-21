#include "serial_port.hpp"

/* SerialPort() //{ */

SerialPort::SerialPort() {
}

//}

/* ~SerialPort() //{ */

SerialPort::~SerialPort() {
  disconnect();
}

//}

/* checkConnected() //{ */

bool SerialPort::checkConnected() {

  struct termios tmp_newtio;
  int            serial_status = tcgetattr(serial_port_fd_, &tmp_newtio);

  if (serial_status == -1) {

    printf("Serial port disconnected\n");
    close(serial_port_fd_);
    return false;
  }

  return true;
}

//}

/* connect() //{ */

bool SerialPort::connect(const std::string port, const int baudrate, const bool virtual_comm) {

  this->virtual_ = virtual_comm;

  // Open serial port
  // O_RDWR - Read and write
  // O_NOCTTY - Ignore special chars like CTRL-C

  serial_port_fd_ = open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);

  if (serial_port_fd_ == -1) {
    printf("could not open serial port %s", port.c_str());
    return false;

  } else {
    fcntl(serial_port_fd_, F_SETFL, 0);
  }

  struct termios newtio;
  bzero(&newtio, sizeof(newtio));  // clear struct for new port settings

  uint16_t baudrate_set;
  switch (baudrate) {
    case 9600: {
      baudrate_set = B9600;
      break;
    }
    case 19200: {
      baudrate_set = B19200;
      break;
    }
    case 38400: {
      baudrate_set = B38400;
      break;
    }
    case 57600: {
      baudrate_set = B57600;
      break;
    }
    case 115200: {
      baudrate_set = B115200;
      break;
    }
    case 230400: {
      baudrate_set = B230400;
      break;
    }
    case 460800: {
      baudrate_set = B460800;
      break;
    }
    case 500000: {
      baudrate_set = B500000;
      break;
    }
    case 576000: {
      baudrate_set = B576000;
      break;
    }
    case 921600: {
      baudrate_set = B921600;
      break;
    }
    case 1152000: {
      baudrate_set = B1152000;
      break;
    }
    case 2000000: {
      baudrate_set = B2000000;
      break;
    }
    default:
      baudrate_set = 0;
      printf("Unsupported baudrate");
      return false;
  }

  cfsetispeed(&newtio, baudrate_set);  // Input port speed
  cfsetospeed(&newtio, baudrate_set);  // Output port speed

  newtio.c_cflag &= ~PARENB;  // no parity bit
  newtio.c_cflag &= ~CSTOPB;  // 1 stop bit
  newtio.c_cflag &= ~CSIZE;   // Only one stop bit
  newtio.c_cflag |= CS8;      // 8 bit word

  newtio.c_iflag = 0;  // Raw output since no parity checking is done
  newtio.c_oflag = 0;  // Raw output
  newtio.c_lflag = 0;  // Raw input is unprocessed

  // |  coppied from MAVROS to possibly fix the issue with arduino  |
  newtio.c_iflag &= ~(IXOFF | IXON);
  newtio.c_cflag &= ~CRTSCTS;
  // | ----------------------------  ---------------------------- |

  newtio.c_cc[VTIME] = 0;  // Wait for up to VTIME*0.1s (1 decisecond), returning as soon as any data is received.
  newtio.c_cc[VMIN]  = 0;

  tcflush(serial_port_fd_, TCIFLUSH);
  tcsetattr(serial_port_fd_, TCSANOW, &newtio);

  setBlocking(serial_port_fd_, 0);

  tcsetattr(serial_port_fd_, TCSANOW, &newtio);

 #if defined(__linux__)
  // Enable low latency mode on Linux
{

  struct serial_struct ser_info;
  ioctl(serial_port_fd_, TIOCGSERIAL, &ser_info);

  ser_info.flags |= ASYNC_LOW_LATENCY;

  ioctl(serial_port_fd_, TIOCSSERIAL, &ser_info);


}
#endif



  return true;
}

//}

/* setBlocking //{ */

void SerialPort::setBlocking(int fd, int should_block) {
  struct termios tty;
  memset(&tty, 0, sizeof tty);
  if (tcgetattr(fd, &tty) != 0) {
    printf("error %d from tggetattr", errno);
    return;
  }

  tty.c_cc[VMIN]  = should_block ? should_block: 0;
  tty.c_cc[VTIME] = 0;  // 0.0 seconds read timeout

  if (tcsetattr(fd, TCSANOW, &tty) != 0)
    printf("error %d setting term attributes", errno);
}

//}

/* disconnect() //{ */

void SerialPort::disconnect() {

  // TODO(lfr) wait for thread to finish
  try {

    close(serial_port_fd_);
  }
  catch (int e) {

    printf("Error while closing the sensor serial line!\n");
  }
}

//}

/* sendChar() //{ */

bool SerialPort::sendChar(const char c) {
  try {
    return write(serial_port_fd_, (const void*)&c, 1);
    if (!virtual_) {
      tcflush(serial_port_fd_, TCOFLUSH);
    }
  }
  catch (int e) {

    printf("Error while writing to serial line!\n");
    return false;
  }
}

//}

/* sendCharArray() //{ */

bool SerialPort::sendCharArray(uint8_t* buffer, int len) {

  try {
    bool ret_val = write(serial_port_fd_, buffer, len);
    if (!virtual_) {
      tcflush(serial_port_fd_, TCOFLUSH);
    }
    return ret_val;
  }
  catch (int e) {

    printf("Error while writing to serial line!\n");
    return false;
  }
}

//}

/* read() //{ */
uint32_t SerialPort::readSerial(uint8_t* arr, uint32_t arr_max_size) {
  
  uint32_t nBytesRead = 0U;

  int readResult = read(serial_port_fd_, arr, static_cast<size_t>(arr_max_size));

  if(0 < readResult)
  {
    nBytesRead = static_cast<uint32_t>(readResult);
  }
  else if(0 == readResult)
  {
    /*End of file reached - let nBytesRead = 0U*/
  }
  else
  {
    /*Error occurred - let nBytesRead = 0U*/
    printf("Error while reading from serial line!\n");
  }

  return nBytesRead;
}

//}