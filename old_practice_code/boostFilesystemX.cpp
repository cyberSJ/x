//#define BOOST_NO_CXX11_SCOPED_ENUMS
#include <boost/filesystem.hpp>
//#undef BOOST_NO_CXX11_SCOPED_ENUMS
#include <iostream>
#include <iterator>
#include <vector>
#include <algorithm>

int main (int argc, char **argv){
	//boost::filesystem::copy_file("a", "b");	// boost is not ABI stable. So linker problem occurs. Solution is to build Boost using c++0x option, and make sure the linker library path is added by the build system.

	//boost::filesystem::path wp("foobar");
	//std::cout << "Path name: " << wp.string() << std::endl;
	//boost::filesystem::create_directory(wp);	// can't link to boost_filesystem library
	
	// Somehow, I can't link to boost_filesystem library.
	//if (boost::filesystem::exists(wp)){
	//	std::cout << "Yes" << std::endl;
	//}

	boost::filesystem::path p("/home/sung/Documents");

	try
	{
		if (boost::filesystem::exists(p))
		{
			if (boost::filesystem::is_regular_file(p))
			{
				std::cout << p << " size is " << 
				boost::filesystem::file_size(p) << std::endl;
			}
			else if (boost::filesystem::is_directory(p))
			{
				std::cout << p << " is a directory containing:\n";

				std::vector<boost::filesystem::path> v;

				std::copy(boost::filesystem::directory_iterator(p),
				boost::filesystem::directory_iterator(),
				std::back_inserter(v));

				std::sort(v.begin(), v.end());

				for (auto content : v)
				{
					std::cout << content << std::endl;
				}
			}
		}
	}
	catch (const boost::filesystem::filesystem_error& ex)
	{
		std::cout << ex.what() << '\n';
	}

	return 0;
}
