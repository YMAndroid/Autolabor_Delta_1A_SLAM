/**********************************************************************************
File name:	  CDeviceConnection.cpp
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

/********************************** File includes *********************************/
#include <CDeviceConnection.h>

/********************************** Current libs includes *************************/

/*********************************** Name space ***********************************/
using namespace std;
using namespace everest;
using namespace everest::hwdrivers;

/********************************** Static varible init ***************************/
bool CDeviceConnection::m_str_map_inited = false;
CStrMap CDeviceConnection::m_str_map;

/***********************************************************************************
Function:     CDeviceConnection
Description:  The constructor of CDeviceConnection
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
CDeviceConnection::CDeviceConnection()
{
	if (!m_str_map_inited)
	{
		m_str_map_inited = true;
		buildStrMap();
	}

	m_dc_port_name = "Unknown port name";
	m_dc_port_type = "Unknown port type";
	m_dc_device_name = "Unknown device type";
}

/***********************************************************************************
Function:     ~CDeviceConnection
Description:  The destructor of CDeviceConnection
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
CDeviceConnection::~CDeviceConnection()
{
	close();
}

/***********************************************************************************
Function:     buildStrMap
Description:
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
void CDeviceConnection::buildStrMap(void)
{
	m_str_map[STATUS_NEVER_OPENED] = "never opened";
	m_str_map[STATUS_OPEN] = "open";
	m_str_map[STATUS_OPEN_FAILED] = "open failed";
	m_str_map[STATUS_CLOSED_NORMALLY] = "closed";
	m_str_map[STATUS_CLOSED_ERROR] = "closed on error";
}

/***********************************************************************************
Function:     buildStrMap
Description:
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
const char * CDeviceConnection::getStatusMessage(int messageNumber) const
{
	CStrMap::const_iterator it;
	if ((it = m_str_map.find(messageNumber)) != m_str_map.end())
	{
		return (*it).second.c_str();
	}
	else
	{
		return NULL;
	}
}

/***********************************************************************************
Function:     buildStrMap
Description:
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
void CDeviceConnection::setPortName(const char *portName)
{
	if (portName != NULL)
	{
		m_dc_port_name = portName;
	}
	else
	{
		m_dc_port_name = "Unknown port name";
	}
}

/***********************************************************************************
Function:     buildStrMap
Description:
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
const char *CDeviceConnection::getPortName(void) const
{
	return m_dc_port_name.c_str();
}

/***********************************************************************************
Function:     buildStrMap
Description:
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
void CDeviceConnection::setPortType(const char *portType)
{
	if (portType != NULL)
	{
		m_dc_port_type = portType;
	}
	else
	{
		m_dc_port_type = "Unknown port type";
	}
}



/***********************************************************************************
Function:     getPortType
Description:
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
const char *CDeviceConnection::getPortType(void) const
{
	return m_dc_port_type.c_str();
}

/***********************************************************************************
Function:     setDeviceName
Description:
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
void CDeviceConnection::setDeviceName(const char *device_name)
{
	if (device_name != NULL)
	{
		m_dc_device_name = device_name;
	}
	else
	{
		m_dc_device_name = "Unknown device name";
	}
}

/***********************************************************************************
Function:     getDeviceName
Description:
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
const char *CDeviceConnection::getDeviceName(void) const
{
	return m_dc_device_name.c_str();
}
