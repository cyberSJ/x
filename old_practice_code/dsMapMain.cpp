#include <map>
#include <iostream>
#include <vector>


/*
	oneCharOff.
	Compute if two strings differe only by one char and
	if they have same length.
	returns bool
*/
bool oneCharOff(const std::string &word1, const std::string &word2){
	if (word1.length() != word2.size()){
		return false;
	}

	int diffs = 0;
	for (int i = 0; i < word1.size(); i++)
		if (word1[i] != word2[i])
			if (++diffs > 1)	return false;

	return diffs == 1;	// should be exactly one char off. diff should not be 0

}


std::map<std::string, std::vector<std::string> > computeAdjacnetWords1(const std::vector<std::string> &words){
	std::map<std::string, std::vector<std::string> > adjWords;

	for (int i = 0; i < words.size(); i++){
		for (int j = i+1; j < words.size(); j++){
			if (oneCharOff(words[i], words[j])){
				adjWords[words[i]].push_back(words[j]);
				adjWords[words[j]].push_back(words[i]);
			}
		}
	}

	return adjWords;
}


int main(int argc, char **argv){

	std::map<std::string, double> salaries;
	salaries["Pat"] = 75000.00;
	std::cout << salaries["Pat"] << std::endl;
	std::cout << salaries["Jan"] << std::endl;
	salaries["Pat"] = 1000.00;

	salaries["Pap"] = 10.00;

	std::map<std::string, double>::const_iterator iter;
	iter = salaries.find("Jan");
	if (iter == salaries.end()){
		std::cout << "Not an employee" << std::endl;
	}
	else{
		std::cout << iter->second << std::endl;
	}

	std::cout << "Two words one char off?: " << oneCharOff(salaries.find("Jan")->first, salaries.find("Pat")->first) << std::endl;
	std::cout << "Pat's salary: " << salaries["Pat"] << std::endl;
	//std::cout << "Two words one char off?: " << oneCharOff(salaries.find("Pbp")->first, salaries.find("Pat")->first) << std::endl;

	return 0;
}
