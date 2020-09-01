/*********************************************************************************
File name:	  CLidarPacket.cpp
Author:       Kimbo
Version:      V1.7.1
Date:	 	  2017-02-04
Description:  lidar packet
Others:       None

History:
	1. Date:
	Author:
	Modification:
***********************************************************************************/

/********************************** File includes *********************************/
#include "CLidarPacket.h"

/********************************** Current libs includes *************************/
#include "C3iroboticsLidarProtocol.h"

/********************************** System includes *******************************/
#include <string.h>
#include <stdio.h>
#include <iostream>

/********************************** Name space ************************************/
using namespace std;
using namespace everest;
using namespace everest::hwdrivers;

/***********************************************************************************
Function:     CLidarPacket
Description:  The constructor of CLidarPacket
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
CLidarPacket::CLidarPacket()
{
	reset();
}

/***********************************************************************************
Function:     reset
Description:  Reset packet
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
void CLidarPacket::reset()
{
    m_buf.resize(m_params.buf_size);
    m_read_length = 0;
    m_length = 0;
    m_valid = false;
}

/***********************************************************************************
Function:     hasWriteCapacity
Description:  Reset packet
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
bool CLidarPacket::hasWriteCapacity(int bytes)
{
	if (bytes < 0)
	{
	    printf("[CLidarPacket] You should input bytes %d less than 0!\n", bytes);
		return false;
	}

	// Make sure there's enough room in the packet
	if ((m_length + bytes) <= m_params.buf_size)
	{
        return true;
	}
	else
	{
		return false;
	}
}

/***********************************************************************************
Function:     pushBack
Description:  pushBack
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
void CLidarPacket::pushBack(u8 ch)
{
	if(!hasWriteCapacity(1))
	{
		return ;
	}
	m_buf[m_length] = ch;
	m_length++;
}

/***********************************************************************************
Function:     calcCheckSum
Description:  Copies the given packets buffer into the buffer of this packet, also
			  sets this length and readlength to what the given packet has
Output:       None
Return:       None
Others:       None
***********************************************************************************/
u16 CLidarPacket::calcCheckSum(u8 *start_bytes, u16 num_bytes)
{
	u8  uchCRCHi = 0xFF;                        // CRC高字节的初始化
	u8  uchCRCLo = 0xFF;                        // CRC低字节的初始化
	u16 uIndex = 0;                             // CRC查找表的指针
	while (num_bytes--)
	{
		uIndex = uchCRCLo ^ *start_bytes++;     // 计算CRC
		uchCRCLo = uchCRCHi ^ auchCRCHi[uIndex];
		uchCRCHi = auchCRCLo[uIndex];
	}
	return (uchCRCHi << 8 | uchCRCLo);
}
/***********************************************************************************
Function:     calcCheckSum_Xor
Description:  Copies the given packets buffer into the buffer of this packet, also
			  sets this length and readlength to what the given packet has
Output:       None
Return:       None
Others:       None
***********************************************************************************/
u16 CLidarPacket::calcCheckSum_Xor(u8 *start_bytes, u16 num_bytes)
{
	u16 checksum = 0;                             // Checksum
	while (num_bytes--)
	{
		checksum += *start_bytes++;     // 计算Checksum
	}
	return checksum;
}

/***********************************************************************************
Function:     verifyCheckSum
Description:  Copies the given packets buffer into the buffer of this packet, also
			  sets this length and readlength to what the given packet has
Output:       None
Return:       None
Others:       None
***********************************************************************************/
bool CLidarPacket::verifyCheckSum(u8 ProtoType)
{
    if (m_length < m_params.least_packet_len)
    {
        m_valid = false;
        return false;
    }

    u8 c2 = m_buf[m_length - 2];
    u8 c1 = m_buf[m_length - 1];
    u16 chksum = (c2 << 8) | c1 ;
    u16 caculate;

	if(ProtoType < 1)
		caculate = calcCheckSum(&m_buf[0], m_length - 2);
	else
		caculate = calcCheckSum_Xor(&m_buf[0], m_length - 2);

#if 0
    printf("[CLidarPacket] m_length %d CRC 0x%X 0x%X, receive CRC %d, caculate CRC %d!\n",
              m_length, c2, c1, chksum, caculate);
#endif
    if (chksum == caculate)
	{
	    m_valid = true;
        return true;
    }
    else
	{
	    m_valid = false;
		return false;
	}
}

/***********************************************************************************
Function:     getParamLength
Description:  Get params data length
Output:       None
Return:       None
Others:       None
***********************************************************************************/
u16 CLidarPacket::getParamLength()
{
    if(isValid())
    {
        return bufToUByte2(&m_buf[6]);
    }
    else
    {
        return 0;
    }
}

/***********************************************************************************
Function:     bufToUByte2
Description:  buf to u16
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
u16 CLidarPacket::bufToUByte2(u8 *src_ptr)
{
    u16 data = (src_ptr[0] << 8) | ((u8)src_ptr[1]);
    return data;
}

/***********************************************************************************
Function:     bufToUByte2
Description:  buf to u16
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
s16 CLidarPacket::bufToByte2(u8 *src_ptr)
{
    s16 data = (src_ptr[0] << 8) | ((s8)src_ptr[1]);
    return data;
}

/***********************************************************************************
Function:     bufToUByte
Description:  buf to u8
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
u8 CLidarPacket::bufToUByte(u8 *src_ptr)
{
    u8 data = src_ptr[0];
    return data;
}

/***********************************************************************************
Function:     printHex
Description:  Buffer to data
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
void CLidarPacket::bufferToData(void *dest_ptr, void *src_ptr, size_t length)
{
    printf("[CLidarPacket] It has not realize now!\n");
}

/***********************************************************************************
Function:     printHex
Description:  printf packet buf as hex format
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
void CLidarPacket::printHex()
{
    printf("[CRobotPacket] length %d, read_length %d!\n",
              m_length, m_read_length);

    for(size_t i = 0; i < m_length; i++)
    {
        printf("buf[%d] = 0x%x!\n", (int)i, (m_buf[i]));
    }
}
