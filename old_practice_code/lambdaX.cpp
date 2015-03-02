#include <stdio.h>

/* Lesson learned:
 Lambda function are used for implementing function that is short and
 something you don't want to use any more.

 [] = capture clause. YOu use it when you don't want to supply 
 argument to the lambda function, but you still want to use it
 inside the lambda function.

 Lambda function are name-less function. But you can define one using
 auto keyword.

 ->int = return type of labmda is int. Explicit return declaration.
*/

int main (int argc, char **argv){
	int capture = 4;
	auto algorithm = [](int i, int capture)->int{ printf("Hello %d\n",i + capture); return i+capture; };

	int result = algorithm(2, capture);
	result++;
	return 0;
}
