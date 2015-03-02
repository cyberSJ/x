#include <stdio.h>
#include <vector>
#include <boost/signals2.hpp>
#include <memory>
#include <functional>

/* Lesson learned:
 Signal is created with the message type that the signal will carry.
 Signal is then connected to an arbitrary class.
 That arbitrary class must have operator() overloaded that can take 
 the signal.
 Signal is sent by calling signal(message), then the arbitrary class
 accepts the signal through the overloaded operator().

 Signals can be pass-by-value or pass-by-reference. You need to 
 choose which one you need, but they do make differences.

 std::move() can move the global content inside the operator()
 std::move() is passing xvalue which is interpreted as lvalue.
 Since the passed lvalue now becomes local to operator, the 
 originally passed global content is emptied.
*/

class MyObject{
public:
	MyObject(int var) : myVar(var) {
		std::vector<int> myVec(4);
		myVec.push_back(1);
		myVec.push_back(2);
		myVec.push_back(3);
		myVec.push_back(4);
	}
	int myVar;
};

class HelloWorld{
public:
	void operator()(MyObject &myObj) const{
		std::vector<int> vec(10);
		vec.push_back(1);
		vec.push_back(2);
		vec.push_back(3);
		vec.push_back(4);
		printf("Hello world %d\n", myObj.myVar);
	}
};

class Filler{
	std::vector<int> m_vec;
public:
	Filler(){}

	void fill(std::vector<int> &gVec){
		printf("I'm here. size of gvec: %d\n", gVec.size());
		for (int i = 0; i < gVec.size()/2; i++){
			gVec[i] = i+5;
			m_vec.push_back(i+5);
		}
		for (int i = 0; i < gVec.size(); i++){
			printf("%d\n", gVec[i]);
		}
	}
	
	std::vector<int> getInternalVec(){
		return m_vec;
	}

};

class Detector{
public:
	Detector(){}

	//void operator()(std::vector<int> data){// 
	//	//auto f1 = std::bind(&Detector::consume, this, std::ref(data));
	//	//
 	//	//f1();
	//	printf("Received data with size: %d\n", data.size());
	//	for (int i = 0; i < data.size(); i++)
	//		printf("%d\n", data[i]);
	//	printf("Modifying data...\n");
	//	data[0] = -4;
	//	for (int i = 0; i < data.size(); i++)
	//		printf("%d\n", data[i]);
	//}

	void operator()(std::shared_ptr<std::vector<int> > data){// IMPORTANT: need pass-by-reference in order to change main()'s vector.
		//auto f1 = std::bind(&Detector::consume, this, std::ref(data));
		//
 		//f1();
		printf("Received shared pointer with size: %d\n", data->size());
		for (int i = 0; i < data->size(); i++)
			printf("%d\n", (*data)[i]);
		printf("Modifying data...\n");
		(*data)[0] = -4;
		for (int i = 0; i < data->size(); i++)
			printf("%d\n", (*data)[i]);
	}

	void operator()(std::vector<int> &data){// IMPORTANT: need pass-by-reference in order to change main()'s vector.
		//auto f1 = std::bind(&Detector::consume, this, std::ref(data));
		//
 		//f1();
		printf("Received data with size: %d\n", data.size());
		for (int i = 0; i < data.size(); i++)
			printf("%d\n", data[i]);
		printf("Modifying data...\n");
		data[0] = -4;
		for (int i = 0; i < data.size(); i++)
			printf("%d\n", data[i]);
	}

	void operator()(int &myInt){
		printf("Original myInt : %d\n", myInt);
		myInt++;
		printf("Changed myInt : %d\n", myInt);
		
	}	
	
	void consume(std::vector<int> &data){
		printf("Received data with size: %d\n", data.size());
		for (int i = 0; i < data.size(); i++)
			printf("%d\n", data[i]);
		printf("Modifying data...\n");
		data[0] = -4;
		for (int i = 0; i < data.size(); i++)
			printf("%d\n", data[i]);

	}
};

/* Practicing sending object as a signal.
 Object is an aribtrary object. The receiver of the signal
 is the one that connects to the signal

 So for in optima, connect(some instantiated consumer of sample buffer).
 The function inside the connect() can be any arbitrary class. Except
 the just needs to have operator() overloaded and operator () accept
 the signal object (again any arbitrary, different than connecting object)
 as an input argument. 
*/
int main(int argc, char **argv){
	boost::signals2::signal<void (MyObject)> sig;
	HelloWorld hello;
	sig.connect(hello);
	MyObject myObj(7);
	sig(myObj);

	// Create a global vector
	std::vector<int> gVec(10);
	std::vector<int> gVec2(10);
	std::shared_ptr<std::vector<int> > pgVec3 = std::make_shared<std::vector<int> >(10);

	// Let the filler fills the vector
	Filler filler;
	filler.fill(gVec);
	filler.fill(gVec2);
	filler.fill(*pgVec3);

	// Signal the filled vector to the detector
	boost::signals2::signal<void (int&)> ready;	// int& means ready(whatever) <-- determines the type of "whatever".
	boost::signals2::signal<void (std::vector<int>&)> ready2;
	boost::signals2::signal<void (std::vector<int>)> ready3;
	boost::signals2::signal<void (std::shared_ptr<std::vector<int> >)> ready4;
	Detector detector;
	ready.connect(detector);
	int myInt = 4;
	ready(myInt);							// myInt should be changed and keep changed.
	ready2.connect(detector);
	ready2(gVec);							// Detector should receive gVec and gVec should now be modified!
	ready3.connect(detector);
	ready3(std::move(gVec2));
	ready4.connect(detector);
	ready4(pgVec3);
	//ready(filler.getInternalVec());


	

	printf("Seeing the gVec after ready2 signal with size: %d\n", gVec.size());
	for (int i = 0; i < gVec.size(); i++)
		printf("%d\n", gVec[i]);

	printf("Seeing the gVec2 after ready3 signal with size: %d\n", gVec2.size());
	for (int i = 0; i < gVec2.size(); i++)
		printf("%d\n", gVec2[i]);

	printf("Seeing the shared pointer pgVec3 after ready4 signal with size: %d\n", pgVec3->size());
	for (int i = 0; i < pgVec3->size(); i++)
		printf("%d\n", (*pgVec3)[i]);
	
	printf("myInt at the end: %d\n", myInt);
	
	return 0;
}

