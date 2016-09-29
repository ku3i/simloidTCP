#include "./unitime.h"


//TODO review this

const UniTime UniTime::startTime = getTime();

struct timezone UniTime::theTimeZone = { 0,0 };

UniTime::UniTime() {
  sec = 0;
  nsec = 0;
  usec = 0;
}

std::string
UniTime::msString(UniTime aTime)
{
  std::ostringstream o;
  o << floor( aTime.usec / 1000) << ",";
  int nks = static_cast<int>(aTime.usec -  floor( aTime.usec / 1000) * 1000);
  o << ( nks < 10 ? "00" : ( nks < 100 ? "0" : "")) << nks << "ms";
  return o.str();
}

