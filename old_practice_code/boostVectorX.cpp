#include <boost/numeric/ublas/vector.hpp>
#include <stdio.h>

int main (int argc, char **argv){
	boost::numeric::ublas::bounded_vector<int, 3> bv;

	bv.insert_element(0,3);
	printf("0th element: %d\n", bv[0]);
	printf("1st element: %d\n", bv[1]);
	printf("2nd element: %d\n", bv[2]);

	return 0;
}
