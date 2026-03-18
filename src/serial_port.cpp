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


  //Set Baudrate
  cfsetispeed(&newtio, baudrate_set);
  cfsetospeed(&newtio, baudrate_set);

  //newtio.c_cflag &= ~PARENB;  // no parity bit
  //newtio.c_cflag &= ~CSTOPB;  // 1 stop bit
  //newtio.c_cflag &= ~CSIZE;   // Only one stop bit
  //newtio.c_cflag |= CS8;      // 8 bit word
  //
  //newtio.c_cflag |= CREAD;   // Enable Receiver
  //newtio.c_cflag |= CLOCAL;  // Ignore Modem Control Lines (DCD) The driver ignores the physical pin state (DCD), assumes the connection is always "Local" and active, and happily processes the ASYNC_LOW_LATENCY timer interrupts every 1ms.
  //
  //newtio.c_iflag = 0;  // Raw output since no parity checking is done
  //newtio.c_oflag = 0;  // Raw output
  //newtio.c_lflag = 0;  // Raw input is unprocessed
  //
  //newtio.c_iflag &= ~(IXOFF | IXON); //Disables special characters (Ctrl+S / Ctrl+Q) used to pause and resume text scrolling in old terminals.
  //newtio.c_cflag &= ~CRTSCTS; //Tells Linux to ignore the RTS (Request to Send) and CTS (Clear to Send) signals.


  //Initialize with standard RAW mode (The "Sledgehammer")
  // This disables: ECHO, ICANON (Canonical mode), ISIG (Signals), 
  // IEXTEN (Extended processing), and clears most processing flags.
  cfmakeraw(&newtio);

  //Hardware Settings (Crucial for physical hardware)
  newtio.c_cflag |= (CLOCAL | CREAD); // Ignore modem lines + Enable Receiver
  newtio.c_cflag &= ~CSTOPB;          // 1 Stop bit
  newtio.c_cflag &= ~CRTSCTS;         // Disable Hardware Flow Control (RTS/CTS)

  //Blocking Read Settings (Keep your existing logic)
  // VMIN = 1: Read call blocks until at least 1 byte is available
  // VTIME = 0: No inter-character timeout (wait forever)
  newtio.c_cc[VMIN]  = 1;
  newtio.c_cc[VTIME] = 0;

  tcflush(serial_port_fd_, TCIFLUSH);
  tcsetattr(serial_port_fd_, TCSANOW, &newtio);

  //setBlocking(serial_port_fd_, 0);

  //tcsetattr(serial_port_fd_, TCSANOW, &newtio);

  // 4. ASSERT DTR & RTS 
  // This tells the FCU: "I am ready, send data now."
  int modem_bits = 0;
  if (ioctl(serial_port_fd_, TIOCMGET, &modem_bits) == 0) 
  {
    modem_bits |= TIOCM_DTR; // Host Ready
    modem_bits |= TIOCM_RTS; // Request To Send
    ioctl(serial_port_fd_, TIOCMSET, &modem_bits);
  }

 #if defined(__linux__)
  // Enable low latency mode on Linux
{

  struct serial_struct serial_info;

  if (ioctl(serial_port_fd_, TIOCGSERIAL, &serial_info) == 0) 
  {
      // Set the LOW_LATENCY flag
      serial_info.flags |= ASYNC_LOW_LATENCY;
      
      if (ioctl(serial_port_fd_, TIOCSSERIAL, &serial_info) < 0) 
      {
          printf("[SerialPort]Failed to set ASYNC_LOW_LATENCY");
      }
      else
      {
          printf("[SerialPort]ASYNC_LOW_LATENCY set successfully");
      }
  }

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