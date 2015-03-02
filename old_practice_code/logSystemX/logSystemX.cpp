#include <boost/test/unit_test.hpp>
#include "severityFormatX.hpp"
#include "loggedCase.hpp"

BOOST_FIXTURE_TEST_SUITE(Suite, LoggedCase)

BOOST_AUTO_TEST_CASE(sungTest)
{
	BOOST_CHECK(2==2);
	SvrChnLog scl;
	BOOST_LOG(scl) << "test test";
}

BOOST_AUTO_TEST_SUITE_END()
