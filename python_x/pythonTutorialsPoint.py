#!/usr/bin/python
import os

class Employee:
    'Classdoc'
    empCount = 0

    def __init__(self, name, salary):
        self.name = name
        self.salary = salary
        Employee.empCount += 1

    def displayCount(self):
        print "Total employee: %d" % Employee.empCount

    def displayEmployee(self):
        print "Name:", self.name, ", Salary:", self.salary

print 'hello'
file_object = open('test_file_to_read.txt', 'r+')
print file_object.name

read_string = file_object.read(10)
print "Read string is:", read_string

position = file_object.tell()
print "Current position:", position

file_object.seek(1, 0)
read_string = file_object.read(10)
print "Read string is:", read_string

file_object.close()

print "Current directory:", os.getcwd()

x = 3
assert (x > 2), "x is not greater than 2"

try:
    nonexisting_file = open("nonexisting_file.txt", 'r')
except IOError:
    print "IOError encountered"
else:
    print "else was hit in the try block"

non_numeric_string = "something"
try:
    some_number = int(non_numeric_string)
except ValueError, arg:
    print "The argument does not contain numbers\n", arg

sungEmployee = Employee("sung", 2000)
youEmployee = Employee("you", 3000)

sungEmployee.displayCount()
youEmployee.displayCount()
sungEmployee.displayEmployee()
youEmployee.displayEmployee()
