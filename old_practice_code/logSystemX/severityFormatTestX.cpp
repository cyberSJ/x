#include "severityFormatX.hpp"
#include <boost/log/trivial.hpp>
#include <boost/log/sources/severity_feature.hpp>
#include <boost/log/sources/severity_logger.hpp>

/* Lessons learned:
*/

int main(int argc, char** argv){
	//BOOST_LOG_TRIVIAL(info) << "info";
	//BOOST_LOG_TRIVIAL(debug) << "debug message";

	init_svr_factory();
	SvrChnLog scLog(boost::log::keywords::channel = "someChannel");
	BOOST_LOG_SEV(scLog, FATAL) << "Hello world!";

	return 0;
}
