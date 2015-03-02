#include <boost/log/core.hpp>
#include <boost/log/keywords/filter.hpp>	// filters.hpp vs. filter.hpp..... and filters.hpp is not under /usr/loca/include but in the /home/Sung/Download/boost1_54/..... man...
#include <boost/log/trivial.hpp>
#include <boost/log/sources/severity_channel_logger.hpp>	// for severity_channel_logger object
#include <boost/log/sources/severity_logger.hpp>
#include <boost/log/expressions.hpp>	// for BOOST_LOG_ATTRIBUTE_KEYWORD()

#include <boost/log/utility/setup/file.hpp>	// for add_log_file	depends on boost_system


// For setting up a sink:
//------------------------------
#include <boost/log/utility/setup/formatter_parser.hpp>	// for format. You also need to use log_setup shared library in cmake.
#include <boost/log/utility/setup/common_attributes.hpp> // for add_common_attributes().
#include <boost/log/utility/setup/console.hpp> // for text_ostream_backend
#include <boost/log/sinks/sync_frontend.hpp> // for locked_backend().
#include <boost/log/sinks/text_ostream_backend.hpp> // for locked_backend().
#include <fstream>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <boost/log/sources/logger.hpp>
#include <boost/log/sources/record_ostream.hpp>
#include <boost/log/sinks/unlocked_frontend.hpp>
#include <boost/log/sinks/frontend_requirements.hpp>

// Formatters:
//----------------
#include <string>
#include <iostream>
#include <stdexcept>
#include <boost/log/expressions/formatters/named_scope.hpp>
#include <boost/format.hpp>
#include <boost/phoenix.hpp>
#include <boost/log/attributes/attribute_name.hpp>
#include <boost/log/utility/value_ref.hpp>
#include <boost/log/utility/formatting_ostream.hpp>
#include <boost/log/utility/manipulators/add_value.hpp>

// Filter formatters:
//----------------------
#include <boost/lexical_cast.hpp>
#include <boost/log/attributes/scoped_attribute.hpp>
#include <boost/log/utility/setup/filter_parser.hpp>


/* Lesson learend:
 How to use severity logger:
 ---------------------------------
 severity_logger implicitly adds an attribute "Severity". 
 This attribute has 'userDefined' type, and it is created upon
 construction of the severity_logger.

 User can create a enum structure of name 'userDefined'.
 'userDefined' can be any name.
 This enum structure is used in the construction of the severity_logger.

 Then user can call BOOST_LOG_SEV() with the constructed
 severity_logger, and one of the members in the enum structure that
 the user named 'userDefined'

 So this is where the need to develop a wrapper around the pure
 boost::log --> The user might want to apply custom severity level.

 severity_logger's default template type is an int. But you can use
 enum instead of int. How? int is a type, enum is also a type, and
 template can be any type. So passing enum as a type is perfectly fine.

 How to use channel_logger:
 --------------------------------
 For channel_logger, you instantiate a channel logger object with
 empty template (meaning you are going to use std::string for defining
 channel name), use operator() of the channel logger to set
 the keywords::channel to "channelName".

 You can use the combined severity_channel_logger.

 enum struct for the severity logger cannot be in the same scope as the 
 creation of the severity logger. I think the enum must be global or
 something...

 When you specify your own severity, the cout still says info...
 Maybe because I didn't specify a sink yet, so boost is using the default
 sink... which says [info] on stdout....


 BOOST_LOG_ATTRIBUTE_KEYWORD():
 -------------------------------
 You use this to add a "placeholder" (placeholder for user-input variable)
 The placeholder is called "keyword" in boost world.
 The keywords can adjust filtering and formatting.
 
 Once you use this, the keyword gets registered to the boost's expression
 as a attribute_keyword. It has a public functions such as
 get_name(), or_throw(), etc. It becomes one of the boost expression
 entities.

 Once it's registered, you can obtain this entity from
 boost:log:tag::yourKeyword. It's like typedef, but the result is
 boost expression's entity.

 But it's important to remember that this keyword is a placeholder,
 not an instance of something.


 ========================================
 Following boost log website:
 ========================================

 Trivial logging with filters:
 -------------------------------
 Core logging setting doesn't affect previously passed log, only future.


 Setting up sinks:
 --------------------
 By default, add_file_log() overwrites the log file.
 Altough cmake finds older version of the boost, since the older version
 still works, the entire program still works.
 
 In order to use the boost::log::keyword::format, you need to include
 the formatter_parser.hpp, AND link log_setup shared library. 

 In order to use TimeStamp, you need to add_common_attributes().
 (and include its header)

 2 Things to checks when using a function:
 	- Include the header
	- If necessary, link the library


 Manual Sink Registration:
 ---------------------------
 In order to control the configuration of the sink, we make a shared_ptr
 to a sink. But before we do that we need to determine the type of the
 sink. In this case it is synchronous_sink (thread-safe).
 Then "get" the backend and then add_stream, which is a output file
 stream to a certain file in our case.
 Then add the sink to the logging core.

 
 Attribute addition:
 ----------------------
 When you add an attribute to a logger, that attribute does not
 necessarily gets shown in the stdout/file/etc. Rather, it becomes
 an "attribute" of the log sentence. Filters and formatters can
 SEE that attribute and USE that attribute. For example, if I
 add an "Dimension" attribute, and then a filter is setup so that
 the filter passes messages only with the Dimension attribute,
 then any message I pass with that logger will pass the filter.


 Formatters:
 -----------------
 They are boosst::log::keywords::format.
 May different formats can be passed to it using operator<<.
 They format how the log should be recorded.


 Using 1 sink pointer, but registering multiple sinks to core:
 -----------------------------------------------------------------
 Seems like boost::log::core::get()->add_sink() MOVES (not copy) 
 the pointer to the log file. So you can keep adding sinks, and
 somehow the core gets the latest formatted sink.


 Custom sink backend:
 ------------------------------
 Custom sink backend are just another user-defined class.
 But it contains overloaded consume(), where the user can specify what to
 do with the record that was passed in to the custom backend.
 In order to register backend, you supply the uder-defined backend as the
 template parameter when creating the sink.
 Then sink is added to the core, and the log information can be fed into
 the sink with the custom backend.


 Multiple backends attachment to a single sink:
 ----------------------------------------------
 You attach it using add_stream() of backend object.


 Formatter_factory:
 ---------------------------
 When formatter parser encounts user-defined attribute,
 formatter parser calls formatter factory.
 Formatter factory constructs formatter
 formatter calls the operator<< in custom class.

 args_map data structure (defined in formatter_factory)
 contains parameters (parameters are general concept of
 things such as capability options for formatter_factory.
 One of such parameters is called "format" which is a string
 that is also compatible (useable) for Boost.Format library.

 Formatter_factory is a class. When the formatter_factory is 
 registered, and when the "format" keyword uses that
 formatter_factory during logging output, formatter_factory
 generates the custom output.
 
 
*/

enum my_severity{	// must be a global enum.
	lowest = -2,
	low = -1,
	mid = 0,
	high = 1,
	highest = 2
};

BOOST_LOG_ATTRIBUTE_KEYWORD(myKeyword, "MyKeywordName", int)	// creates a custom boost attribute_keyword

// Custom sink backend. 
// No multi-threading support(concurrent_feeding is what this means).
class MyBackend:
	public boost::log::sinks::basic_sink_backend< boost::log::sinks::concurrent_feeding >{
public:
	// Automatically called function
	void consume(boost::log::record_view const& rec){
		std::cout << rec[boost::log::expressions::smessage] << std::endl;	// just print the message in the record.
	}
};

// point struct for formatter_parser example
struct point{
	float m_x, m_y;

	point() : m_x(0.0f), m_y(0.0f) {}
	point(float x, float y) : m_x(x), m_y(y) {}
};

// Functoin declarations
bool operator== (point const& left, point const& right);
bool operator!= (point const& left, point const& right);

template< typename CharT, typename TraitsT >
std::basic_ostream< CharT, TraitsT >& operator<< (std::basic_ostream< CharT, TraitsT >& strm, point const& p);
template< typename CharT, typename TraitsT >
std::basic_istream< CharT, TraitsT >& operator>> (std::basic_istream< CharT, TraitsT >& strm, point& p);

// Global operator<< overloading for point struct
template< typename CharT, typename TraitsT >
std::basic_ostream<CharT, TraitsT>& operator<< (std::basic_ostream<CharT, TraitsT>& strm, point const& p){
	strm << "(" << p.m_x << ", " << p.m_y << ")";
	return strm;
}

const float epsilon = 0.0001f;
// Global operator overloading
bool operator== (point const &left, point const &right){
	return (left.m_x - epsilon <= right.m_x && left.m_x + epsilon >= right.m_x) &&
	       (left.m_y - epsilon <= right.m_y && left.m_y + epsilon >= right.m_y);
}
bool operator!= (point const &left, point const &right){
	return !(left == right);
}

// Global operator>> overloading for point struct
template< typename CharT, typename TraitsT >
std::basic_istream< CharT, TraitsT >& operator>> (std::basic_istream< CharT, TraitsT >& strm, point& p){
	if (strm.good()){
		CharT left_brace = static_cast<CharT>(0);
		CharT comma = static_cast<CharT>(0);
		CharT right_brace = static_cast<CharT>(0);

		strm.setf(std::ios_base::skipws);
		strm >> left_brace >> p.m_x >> comma >> p.m_y >> right_brace;	// left_brace becomes '(' after this due to what's in strm.

		// you can tell the ostream that output has failed.
		if (left_brace != '(' || comma != ',' || right_brace != ')')
			strm.setstate(std::ios_base::failbit);		
	}
	return strm;
}

// custom formatter for point object. Used by formatter factory.
class point_formatter{
public:
	typedef void result_type;

	explicit point_formatter(std::string const& fmt) : m_format(fmt){
	}

	void operator() (boost::log::formatting_ostream &strm, boost::log::value_ref<point> const& value){
		if (value){
			point const& p = value.get();
			m_format % p.m_x % p.m_y;
			strm << m_format;
			m_format.clear();
		}
	}

private:
	boost::format m_format;
};

// custom formatter factory just for point_formatter.
class point_formatter_factory :
	public boost::log::basic_formatter_factory<char, point>{
public:
	formatter_type create_formatter(boost::log::attribute_name const& name, args_map const& args){
		args_map::const_iterator it = args.find("format");		// finds "format" parameter in the format string.
		if (it != args.end()){
			// If we can find "format" in the boost's keywords, we create a point_formatter
			return boost::phoenix::bind(point_formatter(it->second), boost::log::expressions::stream, boost::log::expressions::attr<point>(name));
		}
		else
			return boost::log::expressions::stream << boost::log::expressions::attr<point>(name);
	}
};

// Custom filter factory just for point class.
class point_filter_factory :
	public boost::log::filter_factory< char >{
public:
	boost::log::filter on_exists_test(boost::log::attribute_name const& name){
		return boost::log::expressions::has_attr<point>(name);
	}

	boost::log::filter on_equality_relation(boost::log::attribute_name const& name, string_type const& arg){
		return boost::log::expressions::attr<point>(name) == boost::lexical_cast<point>(arg);
	}

	boost::log::filter on_inequality_relation(boost::log::attribute_name const& name, string_type const& arg){
		return boost::log::expressions::attr<point>(name) != boost::lexical_cast<point>(arg);
	}
};

// Example of register_simple_formatter_factory
#if 1
void init_factories(){
	//boost::log::register_simple_formatter_factory<point, char>("MyCoordinates");
	boost::log::register_formatter_factory("MyCoordinates", boost::make_shared<point_formatter_factory>());
	boost::log::register_filter_factory("MyCoordinates", boost::make_shared<point_filter_factory>());
}
#endif

int main(int argc, char **argv){
	// Trivial logging - easiest boost log thing:
	//---------------------------------------------------
//	BOOST_LOG_TRIVIAL(trace) << "A trace severity message";
//
	//	Severity_channel_logger practice:
	//---------------------------------------------------
	boost::log::sources::severity_channel_logger<my_severity> mySCLg(boost::log::keywords::channel = "sungChannel");	//create a logger with severity and channel specified.
	BOOST_LOG_SEV(mySCLg, lowest) << "lowest severity message on sungChannel";
//
//	// BOOST_LOG_ATTIRBUTE_KEYWORD() practice:
//	//---------------------------------------------------
//	BOOST_LOG_TRIVIAL(info) << myKeyword.get_name();	// get name of the custom-registered keyword.
//
//	// Logging to file:
//	//---------------------------------------------------
//	//boost::log::add_file_log("myLog.log");	// boost_system needs not be conflicted with 2 different boost_system versions.
//	//boost::log::sources::logger lg;
//	//BOOST_LOG(lg) << "Hello...Testing myLog";
//
//	// Trivial logging with filters practice:
//	//----------------------------------------------
//	boost::log::core::get()->set_filter(boost::log::trivial::severity >= boost::log::trivial::info);
//	BOOST_LOG_TRIVIAL(trace) << "I'm supposed to be not shown";
//	BOOST_LOG_TRIVIAL(warning) << "I'm supposed to be shown";
//
//	// Setting up sinks:
//	//----------------------
//	boost::log::add_common_attributes();
//	boost::log::add_file_log(
//		boost::log::keywords::file_name = "sample.log",
//		boost::log::keywords::rotation_size = 10 * 1024 * 1024,
//		boost::log::keywords::time_based_rotation = boost::log::sinks::file::rotation_at_time_point(0,0,0),
//		// built-in formatter
//		//boost::log::keywords::format = "[%TimeStamp%]: %Message% %Dimension%");
//		// Lambda-style formatter
//		boost::log::keywords::format = (
//			boost::log::expressions::stream
//			<< boost::log::expressions::attr<std::string>("Dimension")
//			<< " "
//			<< boost::log::expressions::smessage
//		)
//	);
//	BOOST_LOG_TRIVIAL(warning) << "first log to file ever4!";
//
//	// Manual sink registration:
//	//------------------------------
//	typedef boost::log::sinks::synchronous_sink< boost::log::sinks::text_ostream_backend > text_sink;	// frontened = type of sink = in thiscase, synchronous_sink.
//	boost::shared_ptr< text_sink > sink = boost::make_shared< text_sink >();
//	sink->locked_backend()->add_stream(
//		boost::make_shared< std::ofstream >("sample.log"));
//	boost::log::core::get()->add_sink(sink);
//	BOOST_LOG_TRIVIAL(info) << "Second log";

	// Create a logger:
	//---------------------
//	boost::log::sources::logger lg;
//	BOOST_LOG(lg) << "First logger ever!";	// But this log will not be shown to stdout since we defined custom sink above.
//
//	// Adding attribute to a logger:
//	//------------------------------------
//	lg.add_attribute("Dimension", boost::log::attributes::constant< std::string >("4th"));
//	BOOST_LOG(lg) << "Logging with Dimension attribute.";	
//
//	// "Extract" an attribute from a record:
//	//---------------------------------------
//	boost::log::record rec = lg.open_record();
//	typedef boost::log::attributes::constant< std::string > MyDimension;
//	boost::log::value_ref<MyDimension> dimension = boost::log::extract<MyDimension>("Dimension", rec);
//	std::cout << dimension << std::endl;	//
//
//	// Manual unlocked_sink registration:
//	//----------------------------------------
//	typedef boost::log::sinks::unlocked_sink< MyBackend > sink_t;			// create a sink with custom backend.
//	boost::shared_ptr< boost::log::core > core = boost::log::core::get();	// get core and sink and link them.
//	boost::shared_ptr< sink_t > sink1 = boost::make_shared< sink_t >();
//	core->add_sink(sink1);
//	sink1->set_filter(boost::log::expressions::attr< my_severity >("MySeverity") >= mid);
//	boost::log::sources::severity_channel_logger< my_severity > sclg;
//	BOOST_LOG_SEV(sclg, mid) << "Jessica";

	// Sink backends (multiple stream attachments to a single sink):
	//-----------------------------------------------------------------
//	boost::shared_ptr< boost::log::core > core = boost::log::core::get();
//	auto backend =
//		boost::make_shared< boost::log::sinks::text_ostream_backend >();
//	backend->add_stream(
//		boost::shared_ptr< std::ostream >(&std::clog, boost::log::empty_deleter()));
//	backend->add_stream(
//		boost::shared_ptr< std::ostream >(new std::ofstream("sample.log")));
//	backend->auto_flush(true);	// flush stream in every log written. Degrades speed, but good for debugging.
//	typedef boost::log::sinks::synchronous_sink< boost::log::sinks::text_ostream_backend > sink_t;
//	boost::shared_ptr< sink_t > sink = boost::make_shared< sink_t >(backend);
//	//sink->set_formatter(boost::log::expressions::stream <<
//	//                    boost::log::expressions::format_named_scope("Scopes", "%n") << 
//	//					boost::log::expressions::message );
//	core->add_sink(sink);
//	BOOST_LOG_TRIVIAL(info) << "This message should be printed on both cout and a file 2";

//	// Formatter factory & filter facotry practice 
//	//-------------------------------
//	init_factories();
//	boost::log::add_console_log(
//		std::clog, 
//		boost::log::keywords::filter = "%MyCoordinates% = \"(10, 10)\"",
//		boost::log::keywords::format = "%TimeStamp% %MyCoordinates(format=\"{%0.3f; %0.3f}\")% %Message%"
//	);	// define keywords "format" that uses "MyCoordinates" formatter factory. Once it is defined, point_formatter can use find it and apply appropriate formatter. Also, the filter is saying that, when tag is applied and only when the point has value (10,10), it will go to the sink.
//	boost::log::add_common_attributes();
//	boost::log::sources::logger fflg;
//	{
//		// TAG lives in the scope only. Outside the scope, the tag is not in effect.
//		BOOST_LOG_SCOPED_LOGGER_TAG(fflg, "MyCoordinates", point(10, 10));
//		BOOST_LOG(fflg) << "Hello, world with coordinates (10,10)owijfe!~";
//	}
//	{
//		BOOST_LOG_SCOPED_LOGGER_TAG(fflg, "MyCoordinates", point(20, 10));
//		BOOST_LOG(fflg) << "This message will be suppressed by filter (20,20)!";
//	}
//	point p(20,10);
//	BOOST_LOG(fflg) << boost::log::add_value("MyCoordinates", p) << "Hello world";
//
	return 0;
}
