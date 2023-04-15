
Generic Robot Software Libary

This code intends to be used as a library in the sense that no core modifications will be necessary in its use. Good software engineering does not require a redesign of its core structure. Extensive proper usage of the library provides the validation.
Concrete classes can be used directly; abstract classes or interfaces are used by subclassing.
Note that in Python there is no real difference between an interface (having only abstract methods) and an abstract class (having a mix of concrete and abstract methods).

This code intends to be generic towards:
 - motors and sensors
 - simulating or working with a real robot
 - methods to calibrate and estimate the robot's state
 - steering the robot by a predefined command sequence, keyboard, algorithm or log file
 - adding fault handling 
 - logging and replaying
 
A concrete class (from which objects can be instantiated) is shown in gray.
Abstract methods of abstract classes (or interfaces) are defined in italic.
The other methods have a default implementation, but can be overriden to change the default behaviour.
Arrows indicate subclassing. 
Class relations with a black dot at the end means that one class is using the other one internally.

Software engineering achieves a separation of concerns. Consider the state estimation. It is a general problem which is independent of robotics. So it can be defined, implemented and tested independently. The key is to define an interface (EstimationProblem) with all the methods any estimation algorithm might need to solve the problem. For the robot, the subclass RobotModel is then implemented so that the robot's state can be estimated. Note that RobotModel is generic: it can be used for any robot and should not be redefined! 

Some abstract methods are hook methods. Hook methods are used in another method of that class but give the subclass the possibility to define the behavior at that point of the method. It is a strong way of parameterizing behavior.
An example is the abstract increaseOfError()-method of the IterativeEstimationAlg class. It is called within the implementation of solve() (abstract method of EstimationAlg) whenever there is an increase of the error instead of the expected error decrease. The subclass can implement it but should not do it.

Good order to read files:
- State
- Sensors
- Motors
- Robots
- RobotEstimation
- Adapters
