#ifndef MockTodo_HPP
#define MockTodo_HPP

#include "Todo.hpp"
#include "gmock/gmock.h"
#include <iostream>

class MockTodo : public Todo
{
  public:
    MockTodo()
    {
        std::cout << "MockTodo created" << std::endl;
    }

    MOCK_METHOD0(doThis, void());
};

#endif // MockTodo_HPP
