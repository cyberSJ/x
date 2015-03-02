//#include <iostream>
//
//class SomeClass{
//public:
//	int m_mv;
//public:
//	SomeClass(){
//	}
//
//};
//
//SomeClass functionThatReturnsClass(){
//	SomeClass sc;
//	sc.m_mv = 4;
//	return std::move(sc);
//}
//
//int main (int argc, char** argv){
//	SomeClass scMain = functionThatReturnsClass();
//	std::cout << scMain.m_mv << std::endl;
//	return 0;
//}



//#include <iostream>
//#include <cassert>
//
//class A
//{
//public:
//    int state_;
//public:
//    enum {destructed = -2, moved_from, default_constructed};
//
//	A() : state_(default_constructed) {}
//    A(const A& a) : state_(a.state_) {}
//    A& operator=(const A& a) {state_ = a.state_; return *this;}
//    A(A&& a) : state_(a.state_) {a.state_ = moved_from;}
//    A& operator=(A&& a)
//    	{state_ = a.state_; a.state_ = moved_from; return *this;}
//    ~A() {state_ = destructed;}
//
//    explicit A(int s) : state_(s) {assert(state_ > default_constructed);}
//
//	friend std::ostream& operator<<(std::ostream& os, const A& a) {
//		switch (a.state_) {
//	        case A::destructed:
//	            os << "A is destructed\n";
//	            break;
//	        case A::moved_from:
//	            os << "A is moved from\n";
//	            break;
//	        case A::default_constructed:
//	            os << "A is default constructed\n";
//	            break;
//	        default:
//	            os << "A = " << a.state_ << '\n';
//	            break;
//		}
//	    return os;
//	}
//
//	friend bool operator==(const A& x, const A& y) {
//		return x.state_ == y.state_;
//	}
//	
//	friend bool operator<(const A& x, const A& y) {
//		return x.state_ < y.state_;
//	}
//};
//
//#if 0
//A f()
//{
//	A y;
//	return y;
//}
//#endif
//
//#if 1
//A f()
//{
//	A y;
//	A y2(y);
//	return std::move(y2);
//}
//#endif
//
//#if 0
//A&& f()
//{
//	A y;
//	return std::move(y);
//}
//#endif
//
//int main(){
//	//A a = f();
//	A a;
//	std::cout << a; 
//
//	A aMove(std::move(a));
//	std::cout << aMove;
//	return 0;
//}





#include <iostream>
 
struct C {
	C() {}
    C(const C&) { std::cout << "A copy was made.\n"; }
};
  
C f() {
	C c;
	//return C();
	return c;
}
	 
int main() {
	std::cout << "Hello World!\n";
    C obj = f();
}
