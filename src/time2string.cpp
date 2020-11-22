#include "time2string.h"
#include <iomanip>

// Convert a time stamp to a string in format YYYY.MM.DD_hh.mm.ss which can be included in filename or other uses
std::string timeStamp_to_string(boost::posix_time::ptime timeLocal)
{
    std::ostringstream s1;
    s1 << std::setfill('0') << std::setw(4) << timeLocal.date().year() << ".";
    s1 << std::setfill('0') << std::setw(2) << timeLocal.date().month().as_number() << ".";
    s1 << std::setfill('0') << std::setw(2) << timeLocal.date().day() << "_";
    s1 << std::setfill('0') << std::setw(2) << timeLocal.time_of_day().hours() << ".";
    s1 << std::setfill('0') << std::setw(2) << timeLocal.time_of_day().minutes() << ".";
    s1 << std::setfill('0') << std::setw(2) << timeLocal.time_of_day().seconds();
    return s1.str();
}
// Convert current time to a string in format YYYY.MM.DD_hh.mm.ss which can be included in filename or other uses
std::string getCurrentTimestampString()
{
    boost::posix_time::ptime timeLocal = boost::posix_time::second_clock::local_time();
    return timeStamp_to_string(timeLocal);
}
