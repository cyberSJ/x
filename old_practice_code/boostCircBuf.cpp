#define BOOST_CB_DISABLE_DEBUG

#include <stdio.h>
#include <boost/circular_buffer.hpp>
#include <assert.h>

#include <boost/numeric/ublas/vector.hpp>
#include <iostream>

/*
	Boost's circular buffer is not like my intuition.
	The circular buffer size is actual number of elements in the buffer.
	The circular buffer capacity is the capacity.
	The capacity can be set later (not only in instantiation) through
	set_capacity().
*/

int main(int argc, char **argv){
	boost::circular_buffer<int> cb;
	cb.set_capacity(30);

	cb.push_back(1);
	printf("Size: %lu\n", cb.size());
	cb.push_back(2);
	printf("Size: %lu\n", cb.size());
	cb.push_back(3);
	printf("Size: %lu\n", cb.size());

	int a = cb[0];
	int b = cb[1];
	int c = cb[2];
	printf("a = %d\t b = %d\t c = %d\t\n", a, b, c);
	// circular buffer full now. Any more push_back() will (1) OVERWRITE the first element, (2) increment begin and end pointer (DIFFERENT THAN MY INTUITION)

	// Circulat buffer iteration:
	//------------------------------
	printf("iterating through the circular buffer\n");
	for (auto iter = cb.begin(); iter != cb.end(); ++iter){
		printf("*iter = %d\n", *iter);
	}

	cb.push_back(4);
	printf("Size: %lu\n", cb.size());
	cb.push_back(5);
	printf("Size: %lu\n", cb.size());
	a = cb[0];
	b = cb[1];
	c = cb[2];
	printf("a = %d\t b = %d\t c = %d\t\n", a, b, c);
	printf("iterating through the circular buffer\n");
	int count = 1;
	for (auto iter = cb.begin(); iter != cb.end(); ++iter, count++){
		printf("*iter = %d\n", *iter);
		std::cout << count << std::endl;
	}
	std::cout << count << std::endl;
	
	cb.pop_back();		// remove last. 
	cb.pop_front();		// remove first.
	
	int f = cb[0];
	//int g = cb[1];	// what is there now in cb[1]? --> causes an run-time error. Cannot access cb[1]. It's like it doesn't exist. So boost's circular buffer has dynamic size.

	//printf("d = %d\t e = %d\t f = %d\t g = %d\n", d, e, f, g);
	//printf("f = %d\t g = %d\n", f, g);
	printf("f = %d\n", f);

	
	// Test: Definition of invalid iterator.
	boost::circular_buffer<int> cb2(3);
	cb2.push_back(1);
	cb2.push_back(2);
	cb2.push_back(3);

	auto it = cb2.begin();
	printf("*it = %d\n", *it);
	assert(*it == 1);
	
	cb2.push_back(4);	// IMPORTANT: Unless "#define BOOST_CB_DISABLE_DEBUG" is applied, *it now causes core dump.
	printf("*it = %d\n", *it);
	assert(*it == 4);
	it = cb2.begin();	// begin address different. (although valid now)
	printf("*it = %d\n", *it);
	//assert(*it == 4);



	// bounded vector:
	//-------------------------
	boost::numeric::ublas::bounded_vector<int, 3> bv;
	bv[0] = 1;
	bv[1] = 2;
	bv[2] = 3;
	std::cout << "bounded vector: " << bv[0] << std::endl;

	return 0;
}
