#ifndef dsEmployee_H
#define dsEmployee_H

#include <iostream>
#include <string>

class Employee{
	public:
		Employee(){}

		void setValue(const std::string & n, double s){
			m_name = n;
			m_salary = s;
		}
		
		const std::string & getName() const{
			return m_name;
		}
		
		double getSalary() const{
			return m_salary;
		}

		/*
			Print function will be used inside operator<<.
			It takes std::ostream & as a parameter, and that is used
			to pipe the necessary information for printing.
		*/	
		void print(std::ostream & out) const{
			out << m_name << " (" << m_salary << ")";
		}


		/*
			This function is used to compare m_salary with
			other external salary (the salary of rhs employee)
		*/
		bool operator<(const Employee & rhs) const{
			return m_salary < rhs.getSalary();
		}


	private:
		std::string m_name;
		double m_salary;

};

/*
	Find the maximum value among the members
*/
template<typename Comparable>
const Comparable & findMax(std::vector<Comparable> &a){
int maxIndex = 0;

for (int i = 0; i < a.size(); i++){
	if(a[maxIndex] < a[i])
		maxIndex = i;
}

return a[maxIndex];
}

/*
	This global public function overrides the operator<<.
	The function requires two arguments:
		std::ostream & for printing
		const & Employee for content of the print
*/
std::ostream & operator<<(std::ostream &out, const Employee &rhs){
	rhs.print(out);	// Append Employee information to std::ostream object
	return out;	// Return the std::ostream object (that contains the appended data).
}

#endif
