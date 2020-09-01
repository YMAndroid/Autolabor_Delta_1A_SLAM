
#ifndef EVEREST_BASE_CLOGUTILS_H_
#define EVEREST_BASE_CLOGUTILS_H_

#include <string>
#include <vector>
#include <stdio.h>
#include <stdarg.h>
#include <sstream>
#include <sys/stat.h>
#include "CTime.h"

/* Change ofstream to*/
template<class T>
std::string os2String(const T& t)
{
    std::ostringstream ostring;
    ostring << t;
    return ostring.str();
}

// A sprintf-like function for std::string
static std::string format(const char *fmt, ...)
{
	if (!fmt) return std::string("");

	int   result = -1, length = 1024;
	std::vector<char> buffer;
	while (result == -1)
	{
		buffer.resize(length + 10);

		va_list args;  // This must be done WITHIN the loop
		va_start(args,fmt);
		result = vsnprintf(&buffer[0], length, fmt, args);
		va_end(args);

		// Truncated?
		if (result>=length) result=-1;
		length*=2;
	}
	std::string s(&buffer[0]);
	return s;
}

/*---------------------------------------------------------------
			renameFile
---------------------------------------------------------------*/
static bool renameFile( const std::string &oldFileName, const std::string &newFileName)
{
	bool ret_err = 0==rename( oldFileName.c_str(), newFileName.c_str() );

	return ret_err;
}

/*---------------------------------------------------------------
			getFileSize
---------------------------------------------------------------*/
static uint64_t getFileSize(const std::string &fileName)
{
	// The rest of the world:
	struct stat filStat;
	if ( stat( fileName.c_str(), &filStat ) )
			return uint64_t(-1);
	else	return uint64_t(filStat.st_size);
}

/*---------------------------------------------------------------
			renameFileExtension
---------------------------------------------------------------*/
static std::string renameFileExtension(std::string &file_path, std::string new_extension)
{
    if (file_path.size()<2) return (file_path + "." + new_extension);

	size_t i_end = file_path.size()-1;

	int	i= (int)(i_end);
	while (i>0)
	{
		if (file_path[i]=='.')
		{
			std::string the_ext = file_path.substr(0, i) + "." + new_extension;
            return the_ext;
		}
		else
            i--;
	}
	return (file_path + "." + new_extension);
}

/*---------------------------------------------------------------
  Convert a timestamp into this textual form (in local time):
      YEAR/MONTH/DAY,HH:MM:SS.MMM
  ---------------------------------------------------------------*/
static std::string  dateTimeLocalToString(const everest::hwdrivers::TTimeStamp &t)
{
	if (t== INVALID_TIMESTAMP) return std::string("INVALID_TIMESTAMP");

	uint64_t        tmp = (t - ((uint64_t)116444736*1000000000));
    time_t          auxTime = tmp / (uint64_t)10000000;
    unsigned int	secFractions = (unsigned int)( 1000000 * (tmp % 10000000) / 10000000.0 );
    tm  *ptm = localtime( &auxTime );

	if (!ptm) return "(Malformed timestamp)";

	return format(
		"%u/%02u/%02u,%02u:%02u:%02u.%06u",
		1900+ptm->tm_year,
		ptm->tm_mon+1,
		ptm->tm_mday,
		ptm->tm_hour,
		ptm->tm_min,
		(unsigned int)ptm->tm_sec,
		secFractions );
}

#endif // EVEREST_BASE_CLOGUTILS_H_
