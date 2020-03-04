"""
Remember: OOP is a programming paradigm
This is not exclusively a PYTHON CONCEPT

Basicallt, Java, c#, C++ follow OOP principles
So, OOP is transfereable.

Also, an object and an instance are basically the same (In Python)
We INSTANTIATE the OBJECT (instance of the class)
This is how it is used.


Now, PYTHON OOP PRACTICE

----------------------------------------

1. DEFINING A CLASS

"""

class Dog:
	pass

# Pass is used as a place holder where code will go
# Also, use CamelCase notation

# This would be in Python 2.x 
class Dog(object):
	pass

# object specifies the parent class, which we are inheriting from
# This is not neccessary in python 3 as it is the implicit default

# LOOK INTO THAT LATER

***************************************************************************************************

"""
2. INSTANCE ATTRIBUTES

All classes create objects
All objects contain characteristics

An objects INITIALIZE...INITIAL attributes...by giving them a default value
This default value is also known as a STATE

The __init__ method must have one ARGUEMENT as well as the SELF VARIABLE
The self variable refers to the object itself (Dog)

"""

# This is the BLUEPRINT (class) to create Dogs (instances of the class)

class Dog:
	
	def __init__(self, name, age):
		self.name = name
		self.age  = age
		
# each dog has a specific name
# this helps to keep track of different dogs

# So, as previously stated, this just creates the class that defines the dogs
# It is not a dog, but just the attributes of a dog


***************************************************************************************************

"""

3. CLASS ATTRIBUTES

If we wanted to define all dogs as mammals, we could use class attributes

So, while each dog has a unique name and age, every dog will be a mammal

"""

class Dog:
	
	# Class Attribute
	species = 'mammal'
	
	# Initializer / Instance Attributes
	def __init__(self, name, age):
		self.name = name
		self.age = age  


***************************************************************************************************

"""

4. INSTANTIATING OBJECTS

Instantiating = making an instance

More accurately: creating a new, unique instance of a class

"""

class Dog:
	pass

Dog() # The first OBJECT (Dog) has been created or "Instatiated"

Dog() # The second OBJECT (Dog has been created or "Instatiated")

# Now, let's name these puppies
# Also the TYPE of these class instances would be 'Dog'

Geoff = Dog() 
Bill = Dog()

# do a quick test to see if the two dogs are the same?
Geoff == Bill

# -----------------------
# This output: False
# -----------------------

# Not the same, which is good. If they were both called Geoff we would only need one Dog() object
# Now for a more complex example

class Dog:
	
	# Class attribute
 	species = 'mammal'
  
	def __init__(self, name, age):
		self.name = name
		self.age = age

philo = Dog("Philo", 5)
mikey = Dog("Mikey", 6)

print("{} is {} and {} is {}". format(philo.name, philo.age, mikey.name, mikey.age))

if philo.species == "mammal":
	print("{0} is a {1}!".format(philo.name, philo.species))

# Output: 
# ----------------------------

# Philo is 5 and Mikey is 6

# Philo is a mammal!

# ----------------------------

***************************************************************************************************

"""

5. EXCERCISE: 'The Oldest Dog"

Step 1. Instantiate three new dogs

Step 2. Give each dog a different age

Step 3. Write a function called 'get_biggest_number()'

Step 4. This function takes any (*) number of ages as an arguement ---> (*args)

(An agurment takes a data input and will produce the outcome we specify. MAX would mean 'Highest', in this case 'Oldest')

Step 5. See if this outputs the age of the oldest dog

"""

class Dog(object):
	
	species = 'mammal'
	
	def __init__(self, name, age):
		self.name = name
		self.age = age
		
jake = Dog("Jake", 7)
doug = Dog("Doug", 4)
william = Dog("William", 5)

def get_biggest_number(*args):
	return max(args)

print("The oldest dog is {} years old.".format(
		get_biggest_number(jake.age, doug.age, william.age)
))

"""
OUTPUT OF THE ABOVE:
-------------------------------------------------------------------------------

>>> class Dog(object):
...     species = 'mammal'
...     def __init__(self, name, age):
...             self.name = name
...             self.age = age
... 
>>> jake = Dog("Jake", 7)
>>> doug = Dog("Doug", 4)
>>> william = Dog("William", 5)
>>> 
>>> def get_biggest_number(*args):
...     return max(args)
... 
>>> print("The oldest dog is {} years old.".format(
...             get_biggest_number(jake.age, doug.age, william.age)
... ))
The oldest dog is 7 years old.
>>> 

"""

***************************************************************************************************

"""
6. INSTANCE METHODS

instance methods = defined within classes

instance methods = get the contents of an instance

instance methods = used to perform operations with the attributes of our objects

INSTANCE METHODS!!

"""


class Dog(object):
	
	species = 'mammal'
	
	def __init__(self, name, age):
		self.name = name
		self.age = age
		
	def description(self):
		return "{} is {} years old". format(self.name, self. age)

	def speak(self, sound):
		return "{} says {}". format(self.name, sound)

mikey = Dog("Mikey", 6)

print(mikey.description())
print(mikey.speak("HOOOOOOOOooooo-wooooooooooooooOOOOOOOOOOOOOOOOOOOOOOOOO"))

"""
OUTPUT OF THE ABOVE
-----------------------------------------------------------------------------------------

/usr/bin/python
Python 2.7.12 (default, Oct  8 2019, 14:14:10) 
[GCC 5.4.0 20160609] on linux2
Type "help", "copyright", "credits" or "license" for more information.
>>> class Dog(object):
...     species = 'mammal'
...     def __init__(self, name, age):
...             self.name = name
...             self.age = age
...     def description(self):
...             return "{} is {} years old". format(self.name, self. age)
...     def speak(self, sound):
...             return "{} says {}". format(self.name, sound)
... 
>>> mikey = Dog("Mikey", 6)
>>> print(mikey.description())
Mikey is 6 years old
>>> print(mikey.speak("HOOOOOOOOooooo-wooooooooooooooOOOOOOOOOOOOOOOOOOOOOOOOO"))
Mikey says HOOOOOOOOooooo-wooooooooooooooOOOOOOOOOOOOOOOOOOOOOOOOO
>>> 

"""

***************************************************************************************************

"""
7. MODIFYING ATTRIBUTES

Attribute values can change based on some 'behaviour'

"""

class Email:
	
	def __init__(self):
		self.is_sent = False
	def send_email(self):
		self.is_sent = True

my_email = Email()
my_email.is_sent

my_email.send_email()
my_email.is_sent

"""
OUT FOR THE ABOVE:
---------------------------------------------------------------------------------

SyntaxError: invalid syntax
>>> class Email:
...     def __init__(self):
...             self.is_sent = False
...     def send_email(self):
...             self.is_sent = True
... 
>>> my_email = Email()
>>> my_email.is_sent
False
>>> my_email.send_email()
>>> my_email.is_sent
True

"""
***************************************************************************************************

"""
8. PYTHON OBJECT INHERITANCE

Inheritance = A new class takes the attributes and methods of another

Inheritence = A new class is called a child class

Inheritence = The class, which the child inherited from, is called a parent

Note: CHILD classes extend or overide the functionality of the PARENT.

Even though a child INHERITS all the attributes and behaviours from the parent...

...the child, but can also specify a different behaviour to follow

"""

# In Python 2.x: 
# Also see New and Old Style classes
# https://wiki.python.org/moin/NewClassVsClassicClass

class Dog(object):
	pass

# In Python 3, this is the same as:

class Dog:
	pass

# AN EXAMPLE OF INHERITENCE AT WORK
# 'Dog Park Example'

"""
STEP 8.1: PRE-INHERITENCE
"""

class Dog:
	def __init__(self, breed):
		self.breed = breed

spencer = Dog("German Shepard")
spencer.breed

sara = Dog("Boston Terrier")
sara.breed

"""
STEP 8.2: POST-INHERITENCE
"""

# Parent class
class Dog:

	# Class attribute
	species = 'mammal'

	# Initializer / Instance attributes
	def __init__(self, name, age):
		self.name = name
		self.age = age

	# instance method
	def description(self):
		return "{} is {} years old".format(self.name, self.age)

	# instance method
	def speak(self, sound):
		return "{} says {}".format(self.name, sound)


# Child class (inherits from Dog class)
class RussellTerrier(Dog):
	def run(self, speed):
		return "{} runs {}".format(self.name, speed)


# Child class (inherits from Dog class)
class Bulldog(Dog):
	def run(self, speed):
		return "{} runs {}".format(self.name, speed)


# Child classes inherit attributes and
# behaviors from the parent class
jim = Bulldog("Jim", 12)
print(jim.description())

# Child classes have specific attributes
# and behaviors as well
print(jim.run("slowly"))

"""
OUTPUT FROM THE ABOVE

"""

"""
9. PARENT VS CHILD CLASSES

This will introduce a new function:

isinstance()

This determines if an instance...

...is also an instance...

...of a certain parent class.

so, is this instance and instance of a parent?

"""

# Parent class
class Dog:

    # Class attribute
    species = 'mammal'

    # Initializer / Instance attributes
    def __init__(self, name, age):
        self.name = name
        self.age = age

    # instance method
    def description(self):
        return "{} is {} years old".format(self.name, self.age)

    # instance method
    def speak(self, sound):
        return "{} says {}".format(self.name, sound)


# Child class (inherits from Dog() class)
class RussellTerrier(Dog):
    def run(self, speed):
        return "{} runs {}".format(self.name, speed)


# Child class (inherits from Dog() class)
class Bulldog(Dog):
    def run(self, speed):
        return "{} runs {}".format(self.name, speed)


# Child classes inherit attributes and
# behaviors from the parent class
jim = Bulldog("Jim", 12)
print(jim.description())

# Child classes have specific attributes
# and behaviors as well
print(jim.run("slowly"))

# Is jim an instance of Dog()?
print(isinstance(jim, Dog))

# Is julie an instance of Dog()?
julie = Dog("Julie", 100)
print(isinstance(julie, Dog))

# Is johnny walker an instance of Bulldog()
johnnywalker = RussellTerrier("Johnny Walker", 4)
print(isinstance(johnnywalker, Bulldog))

# Is julie and instance of jim?
print(isinstance(julie, jim))