#include <iostream>
#include <boost/geometry.hpp>

int main(int argc, char** argv)
{
	typedef boost::geometry::model::point
	<double, 2, boost::geometry::cs::geographic
	<boost::geometry::degree> > LatLon;

	LatLon ll1(30, 20);
	LatLon ll2(10, 50);

	add_point(ll1, ll2);
	add_point(ll2, LatLon(3000,20));

	std::cout << boost::geometry::get<0>(ll1) << std::endl;
	std::cout << boost::geometry::get<1>(ll1) << std::endl;
	std::cout << boost::geometry::get<0>(ll2) << std::endl;
	std::cout << boost::geometry::get<1>(ll2) << std::endl;

	return 0;
}
