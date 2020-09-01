/**********************************************************************************
File name:	  CDeviceConnection.h
Author:       Shizhe
Version:      V1.6.1
Date:	 	  2016-3-2
Description:  A base class for device connect
Others:       None

History:
	1. Date: 2015-09-15
	Author: Kimbo
	Modification: Refactor this class
***********************************************************************************/
#ifndef EVEREST_LIDAR_DEVICECONNECTION_H_
#define EVEREST_LIDAR_DEVICECONNECTION_H_

/********************************** Current libs includes *************************/

/********************************** System includes *******************************/
#include <string>
#include <map>

namespace everest
{
	namespace hwdrivers
	{
		/*
		   Subclasses of this connection type should call setDCPortType in
		   their constructor, then setDCPortName in their openPort.

		   Classes that use a device connection should call setDCDeviceName
		   with the device the connection is attached too (usually in
		   setDeviceConnection on the device)...

		   Things that read the port should call debugStartPacket whenever
		   they are starting reading a packet...  Then debugBytesRead with the
		   byte count whenever they read or fail a read (0 if a read is
		   failed)...  Then debugEndPacket with a boolean for if the packet
		   was a good one or a bad one and an integer that's postive for the
		   type of packet (if successful) or negative to express why the read
		   failed (if not successful).  For the 'why the read failed' the
		   values should be negative, and should all be unique (anywhere a
		   read can return), preferably with a gap between the values, so that
		   if more debugging is needed there are values in between the ones
		   already there.  Generally this information isn't used or computed,
		   unless the global member CDeviceConnection::debugShouldLog is
		   called to turn it on.
		*/
		typedef std::map<int, std::string> CStrMap;

		class CDeviceConnection
		{
			public:
			  enum Status
			  {
                STATUS_NEVER_OPENED = 1, 	/* Never opened */
                STATUS_OPEN,  				/* Currently open */
                STATUS_OPEN_FAILED, 		/* Tried to open, but failed */
                STATUS_CLOSED_NORMALLY, 	/* Tried to open, but failed */
                STATUS_CLOSED_ERROR 		/* Tried to open, but failed */
			  };

			  /* Constructor */
			  CDeviceConnection();

			  /* Destructor also forces a close on the connection */
			  virtual ~CDeviceConnection();

			  /* Reads data from connection */
			  virtual int read(const char *data, unsigned int size,
							   unsigned int ms_wait = 0) = 0;


			  /* Writes data to connection */
			  virtual int write(const char *data, unsigned int size) = 0;

			  /* Gets the status of the connection, which is one of the enum status.
				 If you want to get a string to go along with the number, use getStatusMessage */
			  virtual int getStatus(void) = 0;

			  /* Gets the description string associated with the status */
			  const char *getStatusMessage(int message_number) const;

			  /* Opens the connection again, using the values from
				 setLocation or a previous open */
			  virtual bool openSimple(void) = 0;

			  /* Closes the connection */
			  virtual bool close(void) { return false; }

			  /* Gets the string of the message associated with opening the device */
			  virtual const char * getOpenMessage(int message_number) = 0;

			  /* Gets the port name */
			  const char *getPortName(void) const;

			  /* Gets the port type */
			  const char *getPortType(void) const;

			  /* Sets the device type (what this is connecting to) */
			  void setDeviceName(const char *device_name);

			  /* Gets the device type (what this is connecting to) */
			  const char *getDeviceName(void) const;

			protected:
				/* Sets the port name */
				void setPortName(const char *port_name);

				/* Sets the port type */
				void setPortType(const char *port_type);

                /* Build string map */
				void buildStrMap(void);

			protected:
				static bool 	m_str_map_inited;
				static CStrMap 	m_str_map;
				std::string 	m_dc_port_name;
				std::string 	m_dc_port_type;
				std::string 	m_dc_device_name;
		};
	}
}
#endif


