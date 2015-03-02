#include <iostream>
#include <fstream>

/* Lesson learned:
 is_open() cannot open the file if it doesn't exist.

 You need allocate memory to the array you are going to use in the
 first argument of ifstream::read(). Otherwise, you get segmentation
 fault. Also, when non-pointer variable (like int) is uninitialized AND
 referenced, and that reference is reinterpreted to another type of 
 a pointer, that pointer does have allocated memory, but contains 
 garbage character.	But that garbage character gets overwritten when
 ifstream::read() overwrites that section of data.

*/

int main(int argc, char** argv){
  	std::ifstream file("fileToRead.txt", std::ios::in|std::ios::binary);
  	std::string line;

  	if(!file.is_open()){
		std::cout<< "file cannot be opened!" << std::endl;
  	}

	// Reading desired number of bytes from the file
	// Read int
	long int intData;	// If I declare a pure long int (instead of a pointer to it), and use reference of it during reinterpret_cast, I get a trailing garbage character (although I read data correctly).
	printf("size of int: %lu\n", sizeof(intData));
	printf("size of reinterpreted to char *: %lu\n", sizeof(reinterpret_cast<char *>(&intData)));
	file.read(reinterpret_cast<char *>(&intData), sizeof(intData));
	printf("Data read: %s\n", reinterpret_cast<char*>(&intData));
	printf("Data read: %d\n", intData);

	// Read char
	char * charData = new char;		// If I don't allocate a memory for it, it generates segmentation fault when I print the char pointer. This is because if I don't allocate any memory, ifstream::read() will put the read bytes to unspecified section of memory, which causes segmentation fault.
	printf("size of char: %lu\n", sizeof(charData));
	file.read(charData, sizeof(charData));
	printf("Data read: %s\n", charData);

	// Reference -> Reinterpret -> Dereference = allocated memory?
	int myInt;
	int myIntWithValue = 50;
	char *myIntChar = reinterpret_cast<char *>(&myInt);
	char *myIntChar2 = reinterpret_cast<char *>(&myIntWithValue);
	printf("Content of the myIntChar: %s\n", myIntChar);	// produces garbage value
	printf("Content of the myIntChar2: %s\n", myIntChar2);	// prints ASCII "2", which is equal to 50 in pure value.

  	//int iterationNumber = 1;
  	//while(!file.eof()){
  	//  std::cout << iterationNumber++;
  	//  std::getline(file, line);
  	//  std::cout << line << std::endl;
  	//}

  	file.close();

  	return 0;
}
