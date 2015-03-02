#include <iostream>
#include <string>
#include "dsList.h"

void forceUsingAccessor(const std::string & str) {
	std::cout << "used accessor: " << str << std::endl;
}

int main(int argc, char** argv){

	dsList<std::string> myList;
	myList.push_back("hello");
	myList.push_back("new");
	myList.push_back("world");
	

	for(dsList<std::string>::const_iterator iter = myList.begin(); iter != myList.end(); ++iter){
	//	std::cout << *iter << std::endl;
		//(*iter)[0] = 'b';	// shouldn't work.



		/* 
			Testing if operator= is shallow copy
			since we didn't define operator=
			and there is Node pointer in the member variable
			of class const_iterator
			Works only if we change the operator* to return 
			non-const Object &
		*/
		//dsList<std::string>::const_iterator iter2 = iter;
		//std::cout << "shallow copy... *iter2 = ++iter iter2: " << *iter2 << "   iter: " << *iter << std::endl;
		//std::cout << "chaniging only iter. iter2 shouldn't change." << std::endl;
		//(*iter)[0] = 'b';	// should work with changes above.
		////++iter;
		//std::cout << "after changing... *iter2 = ++iter iter2: " << *iter2 << "   iter: " << *iter << std::endl;
		//++iter;
		//std::cout << "after incrementing only iter... *iter2 = ++iter iter2: " << *iter2 << "   iter: " << *iter << std::endl;
		// Why does operator++ affect iter, but not iter2?: because iter's m_current now points to the next node, but iter2's m_current still points to the same old node.

	
		/*
			Testing post-fix operator++.
		*/
		//dsList<std::string>::const_iterator iter3 = iter;
		//iter3 = iter++;		// iter3 should be iter and not iter++.
		//std::cout << "iter3 = " << *iter3 << "    iter = " << *iter << std::endl;
		


	}


	for(dsList<std::string>::iterator iter = myList.begin(); iter != myList.end(); ++iter ){
		//forceUsingAccessor(*iter);
		//std::cout << *iter << std::endl;
	 	//*iter == *iter;
		//std::string myObject = *iter;
		//std::cout << "Extracted object: " << myObject << std::endl;
		//*iter = *iter;
		//(*iter)[0] = 'b';
		//std::cout << *iter << std::endl;
		//dsList<std::string>::iterator iter2 = ++iter;
		//iter = ++iter;
		//std::cout << "after iter = ++iter  " << *iter << std::endl;
		//dsList<std::string>::iterator iter4 = iter++;
		//std::cout << "iter4 = " << *iter4 << "      iter = " << *iter << std::endl;
		std::cout << *iter << std::endl;
	}

	dsList<std::string> myList2(myList);
	for(dsList<std::string>::iterator iter = myList2.begin(); iter != myList2.end(); ++iter){
		std::cout << *iter << std::endl;
		dsList<std::string>::iterator emptyIter;
		dsList<std::string>::iterator otherIter = myList.begin();
		//myList2.insert(emptyIter, "I shouldn't be inserted");
		myList2.insert(otherIter, "I shouldn't be inserted");
		
	}

	std::cout << "The front = " << myList2.front() << std::endl;
	std::cout << "The back = " << myList2.back() << std::endl;


	

	
	return 0;
}
