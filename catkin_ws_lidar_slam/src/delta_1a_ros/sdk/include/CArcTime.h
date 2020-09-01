/**********************************************************************************
File name:	  CArcTime.cpp
Author:       Shizhe
Version:      V1.6.2
Date:	 	  2016-3-2
Description:  A class for time readings and measuring durations
Others:       None

History:
	1. Date: 2015-09-23
	Author: Kimbo
	Modification: Refactor this class
***********************************************************************************/
#ifndef EVEREST_LIDAR_CARCTIME_H
#define EVEREST_LIDAR_CARCTIME_H

/********************************** System libs includes **************************/
#include <limits.h>
#include <stddef.h>
#include <sys/time.h>

namespace everest
{
    namespace hwdrivers
    {
        /*
            This class is for timing durations or time between events.
            The time values it stores are relative to an abritrary starting time; it
            does not correspond to "real world" or "wall clock" time in any way,
            so DON'T use this for keeping track of what time it is,
            just for timestamps and relative timing (e.g. "this loop needs to sleep another 100 ms").

            The recommended methods to use are setToNow() to reset the time,
            mSecSince() to obtain the number of milliseconds elapsed since it was
            last reset (or secSince() if you don't need millisecond precision), and
            mSecSince(CArcTime) or secSince(CArcTime) to find the difference between
            two CArcTime objects.

            On systems where it is supported this will use a monotonic clock,
            this is an ever increasing system that is not dependent on what
            the time of day is set to.  Normally for linux gettimeofday is
            used, but if the time is changed forwards or backwards then bad
            things can happen.  Windows uses a time since bootup, which
            functions the same as the monotonic clock anyways.  You can use
            CArcTime::usingMonotonicClock() to see if this is being used.  Note
            that an CArcTime will have had to have been set to for this to be a
            good value... Cia::init does this however, so that should not be
            an issue.  It looks like the monotonic clocks won't work on linux
            kernels before 2.6.
        */
        class CArcTime
        {
            public:
              /* Constructor. Time is initialized to the current time */
              CArcTime() { setToNow(); }

              /* Copy constructor */
              //
              CArcTime(const CArcTime &other) :
                m_Sec(other.m_Sec),
                m_MSec(other.m_MSec)
              {}

              /* Assignment operator */
              CArcTime &operator=(const CArcTime &other)
              {
                if (this != &other) {
                  m_Sec = other.m_Sec;
                  m_MSec = other.m_MSec;
                }
                return *this;
              }

              /* Destructor */
              ~CArcTime() {}

              /// Gets the number of milliseconds since the given timestamp to this one
              long mSecSince(CArcTime since) const
              {
                long long ret = mSecSinceLL(since);
                if (ret > INT_MAX)
                  return INT_MAX;
                if (ret < -INT_MAX)
                  return -INT_MAX;
                return ret;
                  /*  The old way that had problems with wrapping
                  long long timeSince, timeThis;

                  timeSince = since.getSec() * 1000 + since.getMSec();
                  timeThis = m_Sec * 1000 + m_MSec;
                  return timeSince - timeThis;
                  */
              }
          /// Gets the number of milliseconds since the given timestamp to this one
          long long mSecSinceLL(CArcTime since) const
          {
            long long timeSince, timeThis;

            timeSince = since.getSecLL() * 1000 + since.getMSecLL();
            timeThis = m_Sec * 1000 + m_MSec;
            return timeSince - timeThis;
          }
          /// Gets the number of seconds since the given timestamp to this one
          long secSince(CArcTime since) const
          {
            return mSecSince(since)/1000;
          }
          /// Gets the number of seconds since the given timestamp to this one
          long long secSinceLL(CArcTime since) const
          {
            return mSecSinceLL(since)/1000;
          }
          /// Finds the number of millisecs from when this timestamp is set to to now (the inverse of mSecSince())
          long mSecTo(void) const
          {
            CArcTime now;
            now.setToNow();
            return -mSecSince(now);
          }
          /// Finds the number of millisecs from when this timestamp is set to to now (the inverse of mSecSince())
          long long mSecToLL(void) const
          {
            CArcTime now;
            now.setToNow();
            return -mSecSinceLL(now);
          }
          /// Finds the number of seconds from when this timestamp is set to to now (the inverse of secSince())
          long secTo(void) const
          {
            return mSecTo()/1000;
          }
          /// Finds the number of seconds from when this timestamp is set to to now (the inverse of secSince())
          long long secToLL(void) const
          {
            return mSecToLL()/1000;
          }
          /// Finds the number of milliseconds from this timestamp to now
          long mSecSince(void) const
          {
            CArcTime now;
            now.setToNow();
            return mSecSince(now);
          }
          /// Finds the number of milliseconds from this timestamp to now
          long long mSecSinceLL(void) const
          {
            CArcTime now;
            now.setToNow();
            return mSecSinceLL(now);
          }
          /// Finds the number of seconds from when this timestamp was set to now
          long secSince(void) const
          {
            return mSecSince()/1000;
          }
          /// Finds the number of seconds from when this timestamp was set to now
          long long secSinceLL(void) const
          {
            return mSecSinceLL()/1000;
          }
          /// returns whether the given time is before this one or not
          bool isBefore(CArcTime testTime) const
          {
            if (mSecSince(testTime) < 0)
              return true;
            else
            return false;
          }
          /// returns whether the given time is equal to this time or not
          bool isAt(CArcTime testTime) const
          {
            if (mSecSince(testTime) == 0)
             return true;
            else
              return false;
          }
          /// returns whether the given time is after this one or not
          bool isAfter(CArcTime testTime) const
          {
            if (mSecSince(testTime) > 0)
              return true;
            else
              return false;
          }
          /// Resets the time
          void setToNow(void);
          /// Add some milliseconds (can be negative) to this time
          bool addMSec(long ms)
          {
            //unsigned long timeThis;
            long long timeThis;
            timeThis = m_Sec * 1000 + m_MSec;
            //if (ms < 0 && (unsigned)abs(ms) > timeThis)
            if (ms < 0 && -ms > timeThis)
            {
//              CLog::log(CLog::Terse, "CArcTime::addMSec: tried to subtract too many milliseconds, would result in a negative time.");
              m_Sec = 0;
              m_MSec = 0;
              return false;
            }
            else
            {
              timeThis += ms;
              m_Sec = timeThis / 1000;
              m_MSec = timeThis % 1000;
            }
            return true;
          } // end method addMSec

          /// Add some milliseconds (can be negative) to this time
          bool addMSecLL(long long ms)
          {
            //unsigned long timeThis;
            long long timeThis;
            timeThis = m_Sec * 1000 + m_MSec;
            //if (ms < 0 && (unsigned)abs(ms) > timeThis)
            if (ms < 0 && -ms > timeThis)
            {
//              CLog::log(CLog::Terse, "CArcTime::addMSec: tried to subtract too many milliseconds, would result in a negative time.");
              m_Sec = 0;
              m_MSec = 0;
              return false;
            }
            else
            {
              timeThis += ms;
              m_Sec = timeThis / 1000;
              m_MSec = timeThis % 1000;
            }
            return true;
          } // end method addMSec

          /// Sets the seconds value (since the arbitrary starting time)
          void setSec(unsigned long sec) { m_Sec = sec; }
          /// Sets the milliseconds value (occuring after the seconds value)
          void setMSec(unsigned long msec) { m_MSec = msec; }
          /// Gets the seconds value (since the arbitrary starting time)
          unsigned long getSec(void) const { return m_Sec; }
          /// Gets the milliseconds value (occuring after the seconds value)
          unsigned long getMSec(void) const { return m_MSec; }

          /// Sets the seconds value (since the arbitrary starting time)
          void setSecLL(unsigned long long sec) { m_Sec = sec; }
          /// Sets the milliseconds value (occuring after the seconds value)
          void setMSecLL(unsigned long long msec) { m_MSec = msec; }
          /// Gets the seconds value (since the arbitrary starting time)
          unsigned long long getSecLL(void) const { return m_Sec; }
          /// Gets the milliseconds value (occuring after the seconds value)
          unsigned long long getMSecLL(void) const { return m_MSec; }
          /// Logs the time
          void log(const char *prefix = NULL) const
            {
//                CLog::log(CLog::Terse,
//                         "%sTime: %lld.%lld",
//                         ((prefix != NULL) ? prefix : ""),
//                         m_Sec,
//                 m_MSec);
                 }
          /// Gets if we're using a monotonic (ever increasing) clock
          static bool usingMonotonicClock()
          {
        #if defined(_POSIX_TIMERS) && defined(_POSIX_MONOTONIC_CLOCK)
            return ourMonotonicClock;
        #endif
        #ifdef WIN32
            return true;
        #endif
            return false;
          }

          /// Equality operator (for sets)
          bool operator==(const CArcTime& other) const
          {
            return isAt(other);
          }

          bool operator!=(const CArcTime& other) const
          {
            return (!isAt(other));
          }

          // Less than operator for sets
          bool operator<(const CArcTime& other) const
          {
            return isBefore(other);
          } // end operator <

        protected:
          unsigned long long m_Sec;
          unsigned long long m_MSec;
        #if defined(_POSIX_TIMERS) && defined(_POSIX_MONOTONIC_CLOCK)
          static bool ourMonotonicClock;
        #endif

        };
    }
}

#endif
