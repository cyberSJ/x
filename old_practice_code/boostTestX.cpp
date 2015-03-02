#define BOOST_TEST_DYN_LINK	// need this for hand-made makefile
#define BOOST_TEST_MODULE hello 
#include <boost/test/unit_test.hpp>

BOOST_AUTO_TEST_CASE( sungTest ){
	BOOST_CHECK(2 == 3);
}
