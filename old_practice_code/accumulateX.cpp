#include <iostream>
#include <vector>
#include <numeric>
#include <functional>
#include <boost/math/constants/constants.hpp>


/* Lesson learned
	When using std::accumulate, initial value must be double (not int)
	if you want to use double in your summation.

*/

struct pair_sum : public std::plus<std::pair<double,double> >
{
	std::pair<double, double> operator()(const std::pair<double,double> &lhs, const std::pair<double,double> &rhs)
	{
		return std::pair<double,double>(double(lhs.first + rhs.first), lhs.second + rhs.second);
	}
};

std::pair<double,double> mean(std::vector<std::pair<double,double> > &v)
{
	std::pair<double,double> sum = std::accumulate(v.begin(), v.end(), std::make_pair(0.0,0.0), pair_sum());
	sum.first /= v.size();
	sum.second /= v.size();
	return sum;
}

int main(int argc, char** argv)
{
	typedef std::pair<double, double> latlon;
	std::vector<latlon> coordinates;

	coordinates.emplace_back(std::pair<double, double>(1,2));
	coordinates.emplace_back(std::pair<double, double>(2.1,3));
	coordinates.emplace_back(std::pair<double, double>(3,4));
	coordinates.emplace_back(std::pair<double, double>(4,5.3));
	coordinates.emplace_back(std::pair<double, double>(5,6));

	std::pair<double,double> sum = mean(coordinates);
	std::cout << sum.first << ", " << sum.second << std::endl;

	const double pi = boost::math::constants::pi<double>();
	std::cout << pi << std::endl;

	return 0;
}
