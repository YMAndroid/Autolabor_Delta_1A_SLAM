/**********************************************************************************
File name:	  CSerialConnection.cpp
Author:       Shizhe
Version:      V1.6.1
Date:	 	  2016-3-2
Description:  The class is used to receive robot packet
Others:       None

History:
	1. Date: 2015-09-21
	Author: Kimbo
	Modification: Refactor this class
***********************************************************************************/

/********************************** File includes *********************************/
#include "CSerialConnection.h"

/********************************** Current libs includes *************************/
#include "CArcTime.h"
#include "CCountDown.h"

/********************************** System libs includes **************************/
#include <sys/time.h>
#include <termios.h>
#include <unistd.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <errno.h>
#include <iostream>
#include <string.h>

/********************************** Name space ************************************/
using namespace std;
using namespace everest;
using namespace everest::hwdrivers;

#define TIOGETTIMESTAMP         0x5480
#define TIOSTARTTIMESTAMP       0x5481

/***********************************************************************************
Function:     CSerialConnection
Description:  The constructor of CSerialConnection
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
CSerialConnection::CSerialConnection()
{
	m_port = -1;
	m_taking_timestamps = false;
	m_port_name = "none";
	m_baud_rate = 230400;
	m_hardware_control = false;
	m_status = STATUS_NEVER_OPENED;
	buildStrMap();
}

/***********************************************************************************
Function:     CSerialConnection
Description:  The Destructor of CSerialConnection
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
CSerialConnection::~CSerialConnection()
{
	if (m_port != -1)
	{
		close();
	}
}

/***********************************************************************************
Function:     buildStrMap
Description:  Build str map for the serial port error or status.
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
//void CSerialConnection::buildStrMap(void)
//{
//	m_str_map[OPEN_COULD_NOT_OPEN_PORT] = "Could not open serial port.";
//	m_str_map[OPEN_COULD_NOT_SET_UP_PORT] = "Could not set up serial port.";
//	m_str_map[OPEN_INVALID_BAUD_RATE] = "Baud rate invalid, could not set baud on serial port.";
//	m_str_map[OPEN_COULD_NOT_SET_BAUD] = "Could not set baud rate on serial port.";
//	m_str_map[OPEN_ALREADY_OPEN] = "Serial port already open.";
//}

/***********************************************************************************
Function:     getOpenMessage
Description:  Get open message
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
const char* CSerialConnection::getOpenMessage(int messageNumber)
{
    return m_str_map[messageNumber].c_str();
}


int CSerialConnection::set_opt(int fd,int nSpeed, int nBits, char nEvent, int nStop)
{
    struct termios newtio, oldtio;
    if (tcgetattr(m_port,&oldtio) != 0) {
        perror("SetupSerial 1");
        return -1;
    }
    bzero(&newtio, sizeof(newtio));
    newtio.c_cflag |= CLOCAL|CREAD;     //CLOCAL:忽略modem控制线  CREAD：打开接受者
    newtio.c_cflag &= ~CSIZE;           //字符长度掩码。取值为：CS5，CS6，CS7或CS8

    switch(nBits)
    {
        case 7:
            newtio.c_cflag |= CS7;
            break;
        case 8:
            newtio.c_cflag |= CS8;
            break;
    }

    switch( nEvent )
    {
        case 'O':
            newtio.c_cflag |= PARENB; //允许输出产生奇偶信息以及输入到奇偶校验
            newtio.c_cflag |= PARODD;  //输入和输出是奇及校验
            newtio.c_iflag |= (INPCK|ISTRIP); // INPACK:启用输入奇偶检测；ISTRIP：去掉第八位
            break;
        case 'E':
            newtio.c_iflag |= (INPCK|ISTRIP);
            newtio.c_cflag |= PARENB;
            newtio.c_cflag &= ~PARODD;
            break;
        case 'N':
            newtio.c_cflag &= ~PARENB;
            break;
    }

    switch( nSpeed )
    {
        case 2400:
            cfsetispeed(&newtio, B2400);
            cfsetospeed(&newtio, B2400);
            break;
        case 4800:
            cfsetispeed(&newtio, B4800);
            cfsetospeed(&newtio, B4800);
            break;
        case 9600:
            cfsetispeed(&newtio, B9600);
            cfsetospeed(&newtio, B9600);
            break;
        case 115200:
            cfsetispeed(&newtio, B115200);
            cfsetospeed(&newtio, B115200);
            break;
        case 230400:
            cfsetispeed(&newtio, B230400);
            cfsetospeed(&newtio, B230400);
            break;
        case 460800:
            cfsetispeed(&newtio, B460800);
            cfsetospeed(&newtio, B460800);
            break;
        case 1500000:
            cfsetispeed(&newtio, B1500000);
            cfsetospeed(&newtio, B1500000);
            break;
        default:
            cfsetispeed(&newtio, B230400);
            cfsetospeed(&newtio, B230400);
            break;
    }

    if (nStop == 1)
        newtio.c_cflag &= ~CSTOPB; //CSTOPB:设置两个停止位，而不是一个
    else if (nStop == 2)
        newtio.c_cflag |= CSTOPB;

    newtio.c_cc[VTIME] = 0; //VTIME:非cannoical模式读时的延时，以十分之一秒位单位
    newtio.c_cc[VMIN] = 0; //VMIN:非canonical模式读到最小字符数
    tcflush(fd,TCIFLUSH); // 改变在所有写入 fd 引用的对象的输出都被传输后生效，所有已接受但未读入的输入都在改变发生前丢弃。
    if((tcsetattr(m_port,TCSANOW,&newtio))!=0) //TCSANOW:改变立即发生
    {
        perror("com set error");
        return -1;
    }
    printf("set done!\n\r");
    return 0;
}

/***********************************************************************************
Function:     internalOpen
Description:  Open the serial port with the internal serial port
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
int CSerialConnection::internalOpen(void)
{
	struct termios tio;

	if (m_status == STATUS_OPEN)
	{
		printf("[CSerialConnection]Open: Serial port already open");
		return OPEN_ALREADY_OPEN;
	}

	/* Open the port */
	if ((m_port = ::open(m_port_name.c_str(),O_RDWR | O_NDELAY)) < 0)
	{
		printf("[CSerialConnection]Could not open serial port '%s'!\n", m_port_name.c_str());
		return OPEN_COULD_NOT_OPEN_PORT;
    }

	#if 1
    m_status = STATUS_OPEN;
	set_opt(m_port,m_baud_rate, 8, 'N', 1);
	#else

	/* Set the tty baud, buffering and modes */
	if (tcgetattr(m_port, &tio) != 0)
	{
		printf("[CSerialConnection]Could not get port data to set up port!\n");
		close();
		m_status = STATUS_OPEN_FAILED;
		return OPEN_COULD_NOT_SET_UP_PORT;
	}

	/* Turn off echo, canonical mode, extended processing, signals */
	tio.c_lflag &= ~(ECHO | ICANON | IEXTEN | ISIG);

	/* Turn off break sig, cr->nl, parity off, 8 bit strip, flow control */
	tio.c_iflag &= ~(BRKINT | ICRNL | INPCK | ISTRIP | IXON);

	/* Clear size, turn off parity bit */
	tio.c_cflag &= ~(CSIZE | PARENB);

	/* Set size to 8 bits */
	tio.c_cflag |= CS8;

	/* Turn output processing off */
	tio.c_oflag &= ~(OPOST);

	/* Set time and bytes to read at once */
	tio.c_cc[VTIME] = 0;
	tio.c_cc[VMIN] = 0;

	if (tcsetattr(m_port,TCSAFLUSH,&tio) < 0)
	{
		printf("[CSerialConnection]Could not set up port!\n");
		close();
		m_status = STATUS_OPEN_FAILED;
		return OPEN_COULD_NOT_SET_UP_PORT;
	}

	m_status = STATUS_OPEN;

	if (rateToBaud(m_baud_rate) == -1)
	{
		printf("[CSerialConnection]open: Invalid baud rate!\n");
		close();
		m_status = STATUS_OPEN_FAILED;
		return OPEN_INVALID_BAUD_RATE;
	}

	if (!setBaud(m_baud_rate))
	{
		printf("[CSerialConnection]open: Could not set baud rate.!\n");
		close();
		m_status = STATUS_OPEN_FAILED;
		return OPEN_COULD_NOT_SET_BAUD;
	}

	if (!setHardwareControl(m_hardware_control))
	{
		printf("[CSerialConnection]open: Could not set hardware control.!\n");
		close();
		m_status = STATUS_OPEN_FAILED;
		return OPEN_COULD_NOT_SET_UP_PORT;
	}
	#endif // 0

	printf("[CSerialConnection]open: Successfully opened and configured serial port!\n");
	return 0;
}

/***********************************************************************************
Function:     openSimple
Description:  Open the serial port with the internal serial port
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
bool CSerialConnection::openSimple(void)
{
	if (internalOpen() == 0)
	{
		return true;
	}
	else
	{
		return false;
	}
}

/***********************************************************************************
Function:     setPort
Description:  Open the serial port with the internal serial port
Input:        port: The serial port to connect to, or NULL which defaults to
			        COM1 for windows and /dev/ttyS0 for linux
Output:       None
Return:       None
Others:       None
***********************************************************************************/
void CSerialConnection::setPort(const char *port)
{
	if (port == NULL)
	{
		m_port_name = "/dev/ttyS3";
	}
	else
	{
		m_port_name = port;
	}
}

/***********************************************************************************
Function:     closeSerial
Description:  close the serial port with the internal serial port
Input:        None
Output:       None
Return:       The seiral port to connect to
Others:       NoneopenSimple
***********************************************************************************/
void CSerialConnection::closeSerial()
{
    close();
}

/***********************************************************************************
Function:     getPort
Description:  Open the serial port with the internal serial port
Input:        None
Output:       None
Return:       The seiral port to connect to
Others:       None
***********************************************************************************/
const char * CSerialConnection::getPort(void)
{
  return m_port_name.c_str();
}

/***********************************************************************************
Function:     open
Description:  Open the serial port with the internal serial port
Input:        port: The serial port to connect to, or NULL which defaults to
			        COM1 for windows and /dev/ttyS0 for linux
Output:       None
Return:       None
Others:       None
***********************************************************************************/
int CSerialConnection::open(const char *port)
{
	setPort(port);
	return internalOpen();
}

/***********************************************************************************
Function:     close
Description:  Close the serial port
Input:        None
Output:       None
Return:       0 for success, otherwise one of the close enums
Others:       None
***********************************************************************************/
bool CSerialConnection::close(void)
{
	int ret;

	m_status = STATUS_CLOSED_NORMALLY;
	if (m_port == -1)
	return true;

	ret = ::close(m_port);

	if (ret == 0)
	{
		printf("[CSerialConnection]::close: Successfully closed serial port.");
	}
	else
	{
		printf("[CSerialConnection]::close: Unsuccessfully closed serial port.");
	}
	m_port = -1;
	if (ret == 0)
	{
		return true;
	}
	else
	{
		return false;
	}
}

/***********************************************************************************
Function:     setBaud
Description:  Set serial baud
Input:        rate: the baud rate to set the connection to
Output:       None
Return:       whether the set succeeded
Others:       None
***********************************************************************************/
bool CSerialConnection::setBaud(int rate)
{
	struct termios tio;
	int baud;

	m_baud_rate = rate;

	#if 0

	if (getStatus() != STATUS_OPEN)
	return true;

	if ((baud = rateToBaud(m_baud_rate)) == -1)
	return false;

	if (tcgetattr(m_port, &tio) != 0)
	{
		printf("[CSerialConnection]::setBaud: Could not get port data.");
		return false;
	}

	if (cfsetospeed(&tio, baud))
	{
		printf("[CSerialConnection]::setBaud: Could not set output baud rate on termios struct.");
		return false;
	}

	if (cfsetispeed(&tio, baud))
	{
		printf("[CSerialConnection]::setBaud: Could not set input baud rate on termios struct.");
		return false;
	}

	if(tcsetattr(m_port,TCSAFLUSH,&tio) < 0)
	{
		printf("[CSerialConnection]::setBaud: Could not set baud rate.");
		return false;
	}

	startTimeStamping();
	#endif // 0

	return true;
}

/***********************************************************************************
Function:     setBaud
Description:  Start recored the time stamping
Input:        rate: the baud rate to set the connection to
Output:       None
Return:       whether the set succeeded
Others:       None
***********************************************************************************/
void CSerialConnection::startTimeStamping(void)
{
	long baud;
	baud = m_baud_rate;
	if (ioctl(m_port, TIOSTARTTIMESTAMP, &baud) != 0)
	{
		m_taking_timestamps = false;
	}
	else
	{
		m_taking_timestamps = true;
	}
}

/***********************************************************************************
Function:     getBaud
Description:  Start serial baud
Input:        None
Output:       None
Return:       the current baud rate of the connection
Others:       None
***********************************************************************************/
int CSerialConnection::getBaud(void)
{
  return m_baud_rate;
}

/***********************************************************************************
Function:     rateToBaud
Description:  Switch the rate to baud enum
Input:        None
Output:       None
Return:       the baud id
Others:       None
***********************************************************************************/
int CSerialConnection::rateToBaud(int rate)
{
	switch (rate)
	{
		case 300: return B300;
		case 1200: return B1200;
		case 1800: return B1800;
		case 2400: return B2400;
		case 4800: return B4800;
		case 9600: return B9600;
		case 19200: return B19200;
		case 38400: return B38400;
		case 57600: return B57600;
		case 115200: return B115200;
        case 230400: return B230400; 
		default:
		  printf("[CSerialConnection]::rateToBaud: Did not know baud for rate %d.", rate);
		return -1;
	}
}

/***********************************************************************************
Function:     baudToRate
Description:  Switch the baud to rate
Input:        the baud id
Output:       None
Return:       the rate
Others:       None
***********************************************************************************/
int CSerialConnection::baudToRate(int baud)
{
	switch (baud)
	{
		case B300: return 300;
		case B1200: return 1200;
		case B1800: return 1800;
		case B2400: return 2400;
		case B4800: return 4800;
		case B9600: return 9600;
		case B19200: return 19200;
		case B38400: return 38400;
		case B57600: return 57600;
		case B115200: return 115200;
		case B230400: return 230400;
		default:
		  printf("[CSerialConnection]:baudToRate: Did not know rate for baud.");
		return -1;
	}
}

/***********************************************************************************
Function:     setHardwareControl
Description:  hardwareControl true to enable hardware control of lines
Input:        the baud id
Output:       None
Return:       return true if the set succeeded
Others:       None
***********************************************************************************/
bool CSerialConnection::setHardwareControl(bool hardwareControl)
{
	struct termios tio;

	m_hardware_control = hardwareControl;

	if (getStatus() != STATUS_OPEN)
	{
		return true;
	}

	tcgetattr(m_port, &tio);

	/* check for hardware flow control */
	if (m_hardware_control)
	{
		tio.c_cflag |= CRTSCTS;
	}
	else
	{
		tio.c_cflag &= ~CRTSCTS;
	}

	if(tcsetattr(m_port,TCSAFLUSH,&tio) < 0)
	{
		printf("[CSerialConnection]::setHardwareControl: Could not set hardware control.");
		return false;
	}
	else
	{
		return true;
	}
}

/***********************************************************************************
Function:     getHardwareControl
Description:  Get hardware control
Input:        the baud id
Output:       None
Return:       true if hardware control of lines is enabled, false otherwise
Others:       None
***********************************************************************************/
bool CSerialConnection::getHardwareControl(void)
{
	return m_hardware_control;
}

/***********************************************************************************
Function:     write
Description:  Send data
Input:        data: the data to send
			  size: the size of the data to send
Output:       None
Return:       true if hardware write data successfully, false otherwise
Others:       None
***********************************************************************************/
int CSerialConnection::write(const char *data, unsigned int size)
{
	int n;

	if (m_port >= 0)
	{
		n = ::write(m_port, data, size);
		if (n == -1)
		{
			if (errno == EAGAIN)   /* try it again, for USB/serial */
			{
				usleep(10);
				n = ::write(m_port, data, size);
				if (n >= 0)
					return n;
			}
			printf("[CSerialConnection]::write: Error on writing.\n");
			perror("[CSerialConnection]::write:");
		}
		return n;
	}
	printf("[CSerialConnection]::write: Connection invalid.!\n");
	return -1;
}

/***********************************************************************************
Function:     write
Description:  Read data
Input:        data: data buffer for read
			  size: the size of the data to read
			  msWait: overlay time
Output:       None
Return:       true if hardware read data successfully, false otherwise
Others:       None
***********************************************************************************/
int CSerialConnection::read(const char *data, unsigned int size, unsigned int msWait)
{
	struct timeval tp;		/* time interval structure for timeout */
	fd_set fdset;				/* fd set ??? */
	int n;
	long timeLeft;
	unsigned int bytesRead = 0;

    CArcTime timeDone;
	if (m_port >= 0)
	{
		if (msWait >= 0)
		{
            timeDone.setToNow();
            timeDone.addMSec(msWait);
            while ((timeLeft = timeDone.mSecTo()) >= 0)
			{
				tp.tv_sec = (timeLeft) / 1000;	/* we're polling */
				tp.tv_usec = (timeLeft % 1000) * 1000;
				//printf("[CSerialConnection]::read: tp.tv_sec  =%d tp.tv_usec =%d \n",tp.tv_sec ,tp.tv_usec);
				FD_ZERO(&fdset);
				FD_SET(m_port,&fdset);
				int selectResult = select(m_port+1,&fdset,NULL,NULL,&tp);
				if (selectResult <= 0)
				{
				    //printf("[CSerialConnection]::select err bytesRead =%d selectResult = %d errno %d m_port =%d \n",bytesRead ,selectResult,errno,m_port);
					return bytesRead;
				}
				if ((n = ::read(m_port, const_cast<char *>(data)+bytesRead, size-bytesRead)) == -1)
				{
					printf("[CSerialConnection]::read:  Blocking read failed.\n");
					return bytesRead;
				}
				bytesRead += n;
				if (bytesRead >= size)
				{
					return bytesRead;
				}
			}
			return bytesRead;
		}
		else
		{
			n = ::read(m_port, const_cast<char *>(data), size);
			if (n == -1)
			{
				printf("[CSerialConnection]::read:  Non-Blocking read failed.\n");
			}
			return n;
		}
	}
	printf("[CSerialConnection]::read:  Connection invalid.\n");
	return -1;
}

/***********************************************************************************
Function:     getStatus
Description:  Get the serial port status
Input:        None
Output:       None
Return:       return the status the port
Others:       None
***********************************************************************************/
int CSerialConnection::getStatus(void)
{
	return m_status;
}

/***********************************************************************************
Function:     getCTS
Description:
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
bool CSerialConnection::getCTS(void)
{
	unsigned int value;
	if (ioctl(m_port, TIOCMGET, &value) == 0)
	{
		return (bool) (value & TIOCM_CTS);
	}
	else
	{
		perror("ioctl: TIOCMGET");
		return false;
	}
}

/***********************************************************************************
Function:     getDSR
Description:
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
bool CSerialConnection::getDSR(void)
{
	unsigned int value;
	if (ioctl(m_port, TIOCMGET, &value) == 0)
	{
		return (bool) (value & TIOCM_DSR);
	}
	else
	{
		perror("ioctl: TIOCMGET");
		return false;
	}
}

/***********************************************************************************
Function:     getDCD
Description:
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
bool CSerialConnection::getDCD(void)
{
	unsigned int value;
	if (ioctl(m_port, TIOCMGET, &value) == 0)
	{
		return (bool) (value & TIOCM_CAR);
	}
	else
	{
		perror("ioctl: TIOCMGET");
		return false;
	}
}

/***********************************************************************************
Function:     getRing
Description:
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
bool CSerialConnection::getRing(void)
{
	unsigned int value;
	if (ioctl(m_port, TIOCMGET, &value) == 0)
	{
		return (bool) (value & TIOCM_RI);
	}
	else
	{
		perror("ioctl: TIOCMGET");
		return false;
	}
}

