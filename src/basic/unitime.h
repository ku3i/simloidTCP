#ifndef _UNITIME_H_
#define _UNITIME_H_

#include <sys/time.h>
#include <iomanip>
#include <cmath>
#include <string>
#include <iostream>
#include <sstream>

/** search for equivalent of this in std lib and get rid of this */


class UniTime {

  public:
    UniTime();

    /**
    Erzeugt einen UniTimeobjekt mit Sekunden und Nanosekunden
    @param long sec Sekunden,
           long nsec Nanosekunden
    */
    inline UniTime( long, long );
    inline UniTime( const timeval );
    inline UniTime( const timespec );

    long sec;
    long usec;
    long nsec;

    inline timeval c_timeval() const;
    inline timespec c_timespec() const;


    /**
    * Gibt die aktuelle Zeit zurueck.
    * @return UniTime mit aktueller Zeit.
    */
    static inline UniTime getTime();

    /**
    * Operator ueberladen, damit man auch differenz zwischen
    * zwei zeiten berechnen kann. Man sollte nicht eine groessere zeit von einer
    * kleineren abziehen. Wenn jemand lust hat kann er's ja implementieren
    */
    inline UniTime operator - ( const UniTime& t ) const;
    inline UniTime operator + ( const UniTime& t ) const;
    inline UniTime operator * ( const int&     f ) const;
    inline UniTime operator * ( const double&  f ) const;

    inline bool operator <  ( const UniTime& t ) const;
    inline bool operator <= ( const UniTime& t ) const;
    inline bool operator >  ( const UniTime& t ) const;
    inline bool operator >= ( const UniTime& t ) const;
    inline bool operator == ( const UniTime& t ) const;

    /**
    * Zeit seit dem Start des Agenten als String
    * @return string mit der aktuellen Zeit
    */
    static inline const UniTime getTimeStamp();

    /**
    * Zeit zum start des Agenten
    */
    static const UniTime startTime;

    /**
    * formatierte Zeit in ms
    */
    static std::string msString( UniTime aTime);

	/**
	* Zeit in Sekunden (als double)
	*/
	inline double fseconds() const;

  private:

    /**
    * gettimeofday( timeval*, timezone* ) braucht das
    * @see gettimeofday( timeval*, timezone* )
    * Das 'struct' ist wichtig, weil sonst verwechselbar mit typ 'int timezone'!!!
    */
    static struct timezone theTimeZone;
};


inline UniTime::UniTime( long newSec, long newNsec)
    : sec(newSec), usec(newNsec / 1000), nsec(newNsec)
{
}

inline UniTime::UniTime( const timeval t)
    : sec(t.tv_sec), usec(t.tv_usec), nsec(t.tv_usec * 1000)
{
}


inline UniTime::UniTime( const timespec t )
    : sec(t.tv_sec), usec(t.tv_nsec / 1000), nsec(t.tv_nsec)
{
}


inline UniTime UniTime::getTime()
{
  timeval newTime;
  gettimeofday( &newTime, &theTimeZone );
  return UniTime( newTime );
}


inline const UniTime UniTime::getTimeStamp()
{
  return getTime() - startTime;
}


inline UniTime UniTime::operator - ( const UniTime& t ) const
{
  long newSec = sec - t.sec;
  long newNsec = nsec - t.nsec;
  if ( newNsec < 0 ) {
    --newSec;
    newNsec += 1000000000;
  }
  return UniTime( newSec, newNsec );
}

inline UniTime UniTime::operator + ( const UniTime& t ) const
{
  long newSec = sec + t.sec;
  long newNsec = nsec + t.nsec;
  if ( newNsec > 1000000000 ) {
    ++newSec;
    newNsec -= 1000000000;
  }
  return UniTime( newSec, newNsec );
}

inline UniTime UniTime::operator * ( const int& f ) const
{
  long newSec = sec * f;
  long long newNsec = static_cast<long long>(nsec) * f;
  newSec += newNsec / 1000000000;
  newNsec %= 1000000000;
  return UniTime( newSec, newNsec );
}

inline UniTime UniTime::operator * ( const double& f ) const
{
  long double nsecs = sec * 1000000000 + nsec;
  nsecs = nsecs * f;
  const long newSecs = static_cast< long >( nsecs / 1000000000 );
  const long newNsecs = static_cast< long >( nsecs - (newSecs * 1000000000) );
  return UniTime(newSecs, newNsecs);
}

inline bool UniTime::operator < ( const UniTime& t ) const
{
  return ( sec < t.sec || ( sec == t.sec && nsec < t.nsec ) );
}

inline bool UniTime::operator <= ( const UniTime& t ) const
{
  return ( sec < t.sec || ( sec == t.sec && nsec <= t.nsec ) );
}

inline bool UniTime::operator > ( const UniTime& t ) const
{
  return ( sec > t.sec || ( sec == t.sec && nsec > t.nsec ) );
}

inline bool UniTime::operator >= ( const UniTime& t ) const
{
  return ( sec > t.sec || ( sec == t.sec && nsec >= t.nsec ) );
}

inline bool UniTime::operator == ( const UniTime& t ) const
{
  return ( sec == t.sec && nsec == t.nsec );
}

inline std::ostream& operator << (std::ostream& os, const UniTime& tv)
{
  os << tv.sec << "." << std::setfill('0') << std::setw(9) << tv.nsec;

  return os;
}

inline std::ostream& operator << (std::ostream& os, const struct timeval& tv)
{
  os << tv.tv_sec << "." << std::setfill('0') << std::setw(6) << tv.tv_usec;
  return os;
}

inline timeval UniTime::c_timeval() const
{
  timeval tv = { sec, usec };
  return tv;
}

inline timespec UniTime::c_timespec() const
{
  timespec ts = { sec, nsec };
  return ts;
}


inline double UniTime::fseconds() const
{
	return ((double)sec + ((double)nsec/1000000000));
}

#endif
