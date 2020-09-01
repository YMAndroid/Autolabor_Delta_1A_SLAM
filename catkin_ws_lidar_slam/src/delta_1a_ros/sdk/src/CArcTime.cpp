/**********************************************************************************
File name:	  CDeviceConnection.cpp
Author:       Shizhe
Version:      V1.6.2
Date:	 	  2016-3-2
Description:  A base class for device connect
Others:       None

History:
	1. Date: 2015-09-15
	Author: Kimbo
	Modification: Refactor this class
***********************************************************************************/

/********************************** File includes *********************************/
#include "CArcTime.h"

/******************************* System libs includes *****************************/
#include <time.h>

/*********************************** Name space ***********************************/
using namespace everest;
using namespace everest::hwdrivers;

/************************* Static varible init*************************************/
#if defined(_POSIX_TIMERS) && defined(_POSIX_MONOTONIC_CLOCK)
bool      CArcTime::ourMonotonicClock;
#endif

/***********************************************************************************
Function:     CArcTime
Description:  The constructor of CArcTime
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
void CArcTime::setToNow(void)
{
// if we have the best way of finding time use that
#if defined(_POSIX_TIMERS) && defined(_POSIX_MONOTONIC_CLOCK)
    if (ourMonotonicClock)
    {
        struct timespec timeNow;
        if (clock_gettime(CLOCK_MONOTONIC, &timeNow) == 0)
        {
          // start a million seconds into the future so we have some room
          // to go backwards
          m_Sec = timeNow.tv_sec + 1000000;
          m_MSec = timeNow.tv_nsec / 1000000;
          return;
        }
        else
        {
          ourMonotonicClock = false;
//          CLog::logNoLock(CLog::Terse, "CTime::setToNow: invalid return from clock_gettime.");
        }
    }
#endif
    // if our good way didn't work use the old ways
    struct timeval timeNow;

    if (gettimeofday(&timeNow, NULL) == 0)
    {
        // start a million seconds into the future so we have some room
        // to go backwards
        m_Sec = timeNow.tv_sec + 1000000;
        m_MSec = timeNow.tv_usec / 1000;
    }
    else
    {

    }
//    CLog::logNoLock(CLog::Terse, "CTime::setToNow: invalid return from gettimeofday.");
    // thats probably not available in windows, so this is the one we've been using
}
