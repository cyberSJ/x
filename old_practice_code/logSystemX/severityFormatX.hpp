#pragma once
#ifndef SEVERITY_FORMAT_X_HPP
#define SEVERITY_FORMAT_X_HPP

/* Lessons learned:

 Logging with custom Severity:
 --------------------------------
 Depending on whether you use expressions:: attr<>() or attribute keyword, 
 you need to overload formatting_ostream& operator<< 
 OR std::ostream& operator<<, respectively.
 1. Define enum, specifying severities.
 2. Overload formatting_ostream& operator<< if using expressions::attr<>().
    OR overload std::ostream& operator<< if using attribute keyword.
	- This specifies how we are going to log the severity.
 3. If using attribute keyword approach, then register the 
 	keyword "Severity".
 4. Make the sink format include "Severity" tag. 
 5. Declare an instance of a logger that uses severity.
 6. Use the logger with one of the enums
 * I don't think register_simple_formatter_factory approach is working.
   So just follow the above instructions.

 More about ATTRIBUTE_KEYWORD:
 --------------------------------
 BOOST_LOG_ATTRIBUTE_KEYWORD(keyword, "AttributeName", typeOfKeyword)
 When the "keyword" is registered to "AttributeName", whenever the 
 formatter or filter encounters the "keyword", I think boost accesses
 the value stored in the "AttributeName". So "AttributeName" is the
 central hub and "keyword" just typedefs the "AttributeName".
 "AttributeName" is the real attribute. "keyword" is just another name
 for it so that user can use it easily during coding.

*/

#include <iostream>
#include <boost/log/utility/setup/formatter_parser.hpp>
#include <boost/log/utility/setup/common_attributes.hpp>
#include <boost/log/sources/logger.hpp>
#include <boost/log/sources/severity_channel_logger.hpp>
#include <boost/log/sources/record_ostream.hpp>
#include <boost/log/utility/setup/console.hpp>
#include <boost/format.hpp>
#include <boost/log/utility/formatting_ostream.hpp>
#include <boost/log/utility/value_ref.hpp>
#include <boost/phoenix.hpp>
#include <boost/log/utility/manipulators/to_log.hpp>
#include <boost/log/expressions/keyword_fwd.hpp>
#include <boost/log/expressions/keyword.hpp>

// Introduce a custom concept of severity.
enum MySeverity
{
	INFO,
	DEBUG,
	WARNING,
	ERROR,
	FATAL
};

// Let boost know about "severity" keyword
BOOST_LOG_ATTRIBUTE_KEYWORD(severity, "Severity", MySeverity)

// Empty argument required for to_log_manip class.
struct emptyArg;

// Formatting used when attribute keyword is used instead of expressions::attr
std::ostream& operator<< (std::ostream& strm, MySeverity level)
{
	static const char* svrTypes[] =
	{
		"NORM",
		"NOTIFY",
		"WARN",
		"ERROR",
		"CRIT"
	};

	if (static_cast<std::size_t>(level) < sizeof(svrTypes)/sizeof(*svrTypes))
	{
		strm << svrTypes[level];
	}
	else
	{
		strm << static_cast<int>(level);
	}

	return strm;
}

// Formatting used when expressions::attr is used instead of attribute keyword
boost::log::formatting_ostream& operator<<
(
	boost::log::formatting_ostream& strm,
	boost::log::to_log_manip<MySeverity, emptyArg> const& manip
)
{
    static const char* svrTypes[] =
	{
		"NORM",
		"NOTIFY",
		"WARN",
		"ERROR",
		"CRIT"
	};

	MySeverity level = manip.get();
	if (static_cast<std::size_t>(level) < sizeof(svrTypes)/sizeof(*svrTypes))
	{
		strm << svrTypes[level];
	}
	else
	{
		strm << static_cast<int>(level);
	}

	return strm;
}

// Above operator will be taken care by Boost.Log Formatter.
void init_svr_factory()
{
	boost::log::add_console_log
	(
		std::clog,
		boost::log::keywords::format =
		(
			boost::log::expressions::stream <<
			boost::log::expressions::attr<unsigned int>("LineID") <<
			": <" << boost::log::expressions::
			attr<MySeverity, emptyArg>("Severity") << "> " <<
			"<" << severity << "> " <<
			boost::log::expressions::smessage
		)
	);
	boost::log::add_common_attributes();
}

// After initializing, user can instantiate this logger source to log things.
typedef boost::log::sources::
severity_channel_logger<MySeverity, std::string> SvrChnLog; 


BOOST_LOG_ATTRIBUTE_KEYWORD(channel, "Channel", std::string);

#endif
