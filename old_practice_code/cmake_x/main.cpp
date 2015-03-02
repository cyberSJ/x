#include <iostream>
#include "Todo.hpp"
#include "MockTodo.hpp"
//#include "gtest/gtest.h"  // you can choose to include gtest with some
                            // code removed.
#include "gmock/gmock.h"

TEST(todoTestCase, constructor_test)
{
    Todo todo;
}

TEST(todoTestCase2, doThis_test)
{
    Todo todo;

    // Error: EXPECT_CALL() only works with GMOCK functions.
    //EXPECT_CALL(todo, doThis())
    //    .Times(1);

    todo.doThis();
}

TEST(mockTodoTestCase, doThis_test)
{
    MockTodo mock_todo;

    EXPECT_CALL(mock_todo, doThis())
        .Times(2);

    // MockTodo doesn't actually execute the content of doThis(). It just
    // calls a bogus function. So Todo::doThis() is not called although it
    // looks like it is. The actual function called, is GMOCK-generated
    // function called gmock_doThis().
    mock_todo.doThis();
    mock_todo.doThis();
}

int main(int argc, char** argv)
{
    argc = argc;
    argv = argv;

    std::cout << "hello world" << std::endl;


    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
