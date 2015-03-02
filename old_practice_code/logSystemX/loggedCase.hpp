#pragma once
#ifndef LOGGEDCASE_HPP
#define LOGGEDCASE_HPP

/* Lessons Learned:
 린들리는 official document에 나오지 않는 이상한 기술까지도 알아내서
 잘 쓰고 있어.. 신기해..아마도 reference 섹션을 이용한것 같다.

*/

#include <boost/test/unit_test.hpp>

// Sink
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <boost/log/sinks/sync_frontend.hpp>
//#include <boost/log/sinks/text_ostream_backend.hpp>
#include <boost/log/sinks/text_file_backend.hpp>
#include <boost/log/expressions/keyword.hpp>
#include <string>

#include "severityFormatX.hpp"


BOOST_LOG_ATTRIBUTE_KEYWORD(casename, "CaseName", std::string)
BOOST_LOG_ATTRIBUTE_KEYWORD(suitename, "SuiteName", std::string)

struct LoggedCase
{
	std::string m_suite;
	std::string m_case;

	boost::shared_ptr<boost::log::core> m_core;
	typedef boost::log::sinks::synchronous_sink<boost::log::sinks::text_file_backend> text_sink;
	boost::shared_ptr<text_sink> m_sink;
	boost::shared_ptr<boost::log::core> m_logging_core;

public:
	LoggedCase()
	: 	m_case(boost::unit_test::framework::current_test_case().p_name),
	    m_suite(boost::unit_test::framework::get
		<boost::unit_test::test_suite>
	    (boost::unit_test::framework::current_test_case().p_parent_id).
		p_name),
		m_core(boost::log::core::get()),
		m_sink(boost::make_shared<text_sink>()),
		m_logging_core(boost::log::core::get())
	{
		m_core->add_global_attribute(casename.get_name(), 
		boost::log::attributes::constant<casename_type::value_type>
		(m_case));

		m_core->add_global_attribute(suitename.get_name(),
		boost::log::attributes::constant<suitename_type::value_type>
		(m_suite));

		auto backend = m_sink->locked_backend();
		backend->set_file_name_pattern(m_suite + ".log");

		m_sink->set_formatter(
		boost::log::expressions::stream <<
		//casename << " <" << boost::log::keywords::channel << "> (" <<
		casename << " ---- " <<
		boost::log::expressions::smessage);

		m_sink->set_filter(m_suite == suitename);

		m_logging_core->add_sink(m_sink);
	}

};

#endif 
