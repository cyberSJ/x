#include "dsClassSyntax.h"

IntCell::IntCell(){

}

IntCell::IntCell(int initialValue=0):m_storedValue(initialValue){
	//m_storedValue = initialValue;
}

IntCell::~IntCell(){

}

IntCell::IntCell(const IntCell &rhs):m_storedValue(rhs.m_storedValue){

}

const IntCell & IntCell::operator=(const IntCell &rhs){
	if (this != &rhs)	// Avoids copying. using just rhs is valid
		m_storedValue = rhs.m_storedValue;
	return *this;
}

int IntCell::read() const{
	//m_storedValue = 99;	// error: assignment of member in read-only object.
	return m_storedValue;
}

void IntCell::write(int x){
	m_storedValue = x;
}

