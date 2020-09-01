/**********************************************************************************
File name:	  CCountDown.cpp
Author:       Kimbo
Version:      V1.5.0
Date:	 	  2016-4-25
Description:  Time class
Others:       None

History:
	1. Date:
	Author:
	Modification:
***********************************************************************************/

/********************************** File includes *********************************/
#include "CCountDown.h"

/******************************* Current libs includes ****************************/
#include "CTime.h"

/******************************* System libs includes *****************************/
#include <iostream>
#include <stdio.h>

/*********************************** Name space ***********************************/
using namespace std;
using namespace everest;
using namespace everest::hwdrivers;

/***********************************************************************************
Function:     CCountDown
Description:  The constructor of Count down count
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
CCountDown::CCountDown()
{
    m_end_time = INVALID_TIMESTAMP;
    m_end_flag = false;
    m_time_ms = 0.0;
}

/***********************************************************************************
Function:     CCountDown
Description:  The constructor of Count down count
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
CCountDown::CCountDown(double time_ms)
{
    m_end_time = INVALID_TIMESTAMP;
    m_end_flag = false;
    setTime(time_ms);
}

/***********************************************************************************
Function:     ~CCountDown
Description:  The destructor of file path class
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
CCountDown::~CCountDown()
{

}

/***********************************************************************************
Function:     setTime
Description:
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
void CCountDown::setTime(double time_ms)
{
    m_end_flag = false;
    m_time_ms = time_ms;
    m_end_time = CTime::addTime(CTime::getCpuTime(), time_ms);
}

/***********************************************************************************
Function:     isEnd
Description:
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
bool CCountDown::isEnd() const
{
    return CTime::getCpuTime() > m_end_time? true: false;
}

/***********************************************************************************
Function:     getLeftTime
Description:  Get left time, unit is ms
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
double CCountDown::getLeftTime() const
{
    if(m_end_time != INVALID_TIMESTAMP)
    {
        return isEnd()? 0 : CTime::timeDifference(CTime::getCpuTime(), m_end_time);
    }
    else
    {
        printf("[CCountDown] end time is INVALID_TIMESTAMP!\n");
        return -1.0;
    }
}


