#ifndef TIME2STRING_H
#define TIME2STRING_H

#include <boost/date_time.hpp>

// Convert a time stamp to a string in format YYYY.MM.DD_hh.mm.ss which can be included in filename or other uses
std::string timeStamp_to_string(boost::posix_time::ptime timeLocal);
// Convert current time to a string in format YYYY.MM.DD_hh.mm.ss which can be included in filename or other uses
std::string getCurrentTimestampString();
#endif