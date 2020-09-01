/**********************************************************************************
File name:	  CSimulateSerial.h
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
#ifndef EVEREST_HWDRIVERS_CSIMULATESERIAL_H_
#define EVEREST_HWDRIVERS_CSIMULATESERIAL_H_

/********************************** Current libs includes *************************/
#include <CDeviceConnection.h>

/********************************** System libs includes **************************/
#include <string>
#include <fstream>

namespace everest
{
	namespace hwdrivers
	{
		/* For connecting to devices through a serial port */
		class CSimulateSerial: public CDeviceConnection
		{
			public:
				/* Constructor */
				CSimulateSerial();

				/* Destructor also closes the connection */
				virtual ~CSimulateSerial();

				/* Opens simulate, input file path */
				int open(const char * port = NULL);

				/* Sets the port this connection will use */
				void setPort(const char *port = NULL);

				/* 	Gets the port this is using */
				const char* getPort(void);

				virtual bool openSimple(void);

				virtual int getStatus(void);

				virtual bool close(void);

				virtual int read(const char *data, unsigned int size, unsigned int msWait = 0);

				virtual int write(const char *data, unsigned int size);

				virtual const char* getOpenMessage(int messageNumber) { return m_str_map[messageNumber].c_str(); }

				/* Internal open, for use by open and openSimple */
				int internalOpen(void);

				/* Sets the baud rate on the connection */
				bool setBaud(int baud);

				/* Gets what the current baud rate is set to */
				int getBaud(void);

				enum Open
				{
					OPEN_COULD_NOT_OPEN_PORT = 1,  	/* Could not open the port */
					OPEN_COULD_NOT_SET_UP_PORT, 	/* Could not set up the port */
					OPEN_INVALID_BAUD_RATE, 		/* Baud rate is not valid */
					OPEN_COULD_NOT_SET_BAUD, 		/* Baud rate valid, but could not set it */
					OPEN_ALREADY_OPEN 				/* Connection was already open */
				};

			protected:
				/* Change rate to baud, these both return -1 for errors */
				int rateToBaud(int rate);

				/* change baud to rate, these both return -1 for errors  */
				int baudToRate(int baud);

				/* this just tries */
				void startTimeStamping(void);

			protected:
                CStrMap 		m_str_map;
				std::string 	m_port_name;
				int 			m_baud_rate;
				int 			m_status;
				std::ifstream   m_fp;
		};
	}
}
#endif
