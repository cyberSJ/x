class MyClass:
    "This seems like a documentation"
    member_var1 = "I'm a member var 1"

    # Python needs to pass in the "self" object when calling __init__(). 
    def __init__(self, my_name, my_age):
        self.name = my_name
        self.age = my_age

    # Python also needs self. Python needs self in every function. Why...?
    def toString(self):
        print self.name 
        print self.age
        print self.member_var1

myClass = MyClass("sung", 28)
myClass.toString()
