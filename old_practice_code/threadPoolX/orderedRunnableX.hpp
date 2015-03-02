#pragma once
#ifndef ORDEREDRUNNABLEX_HPP
#define ORDEREDRUNNABLEX_HPP

/* Lesson Learend:
 Someone outside the thread pool must create a unique pointer to
 a runnable task function (or class).
 Order can be int, float, or any thing...Therefore it's a template arg.
 Runnable can't take any type.
 You can use initializer list even in the copy constructor.
 Protected members can be accessed inside another instance of the same
 class.
 std::unique_ptr already has operator bool() overloaded. We can use 
 that to overload this class's overator bool().
 You have to return *this in operator= overloading.
 std::move does not violate const reference-ness.

 protected members can be accessed in move constructor. Even private.

 mutable keyword allows the OrderedRunnableX to be moveable even
 in const rvalue referenced object.

 operator= overloading must assign member variables (not construct
 them) from the rhs.

*/

#include <memory>
#include "runnableX.h"

template<typename Order>
class OrderedRunnableX{
protected:
	mutable std::unique_ptr<RunnableX> m_runnable;	
	Order m_order;

public:
	OrderedRunnableX(std::unique_ptr<RunnableX> runnable, Order order)
		: m_runnable(std::move(runnable)),
		  m_order(order)
	{}

	OrderedRunnableX(const OrderedRunnableX&& orig)
		: m_runnable(std::move(orig.m_runnable)),
		  m_order(orig.m_order)
	{}
	
	OrderedRunnableX& operator=(const OrderedRunnableX&& orig){
		// These 2 lines don't work.
		//m_runnable(std::move(orig.m_runnable));	// because m_runnable is a unique pointer and you can't just construct a unique pointer like this.
		//m_order(orig.m_order);	// because m_order is an arbitrary type, so m_order doesn't have enough signature to take m_order as an argument.
		m_runnable = std::move(orig.m_runnable);
		m_order = orig.m_order;		// correct assuming m_order has operator= supplied.
		return *this;
	}

	operator bool() const{
		//TODO: return just m_runnable.
		//return m_runnable;
		return static_cast<bool>(m_runnable); // static_cast just in case?
	}

	bool operator< (const OrderedRunnableX& rhs){
		return m_order < rhs.m_order;
	}

	void operator()(){
		(*m_runnable)();
	}

	const Order& getOrder() const{
		return m_order;
	}
};


#endif
