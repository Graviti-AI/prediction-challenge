# Documentation for custom class derived from `Controller`

## Header files

Please include `Controller/Controller.hpp` before you derive your custome class from `Controller`. 
```C++
#include "Controller/Controller.hpp"
class MyController : public Controller {
    // definition body ...
}
```

## Construction

You are supposed to specify the dimension of `state` vector and `input` vector in the argument list of the construction of the base class `Planner`. 
For example, if we want to derive a `HumanCarPlanner` class, we should write the construction as follows:
```C++
MyController::MyController(/* your argument list here */) : Planner(d, 3) {
    // construction body ...
}

```
since we have an intermediate vector of dimension 3 (gas pedal, brake pedal, steering wheel angle), and an input vector of dimension d (the `input` vector is the output of Zeji's planner).

## Overriding the virtual method

All the interface of a `Controller` lies in the virtual method `update`. The signature of this function is:
```C++
typedef std::vector<double> Vector;
Vector HumanCarController::update(Vector input);
```

It takes `input` vector, which is the output of Zeji's planner. The return vector should be the intermediate vector, which is gas pedal, brake pedal, and steering angle.

For each agent, there is a `getState()` method, which gives the state vector of this agent.
For a car, this state vector has 6 dimensions: location x, location y, yaw angle, speed x, speed y, angular velocity of yaw.
You can use `agents` to get information of the environment.

## Exception handling

If the dimensions of the vectors do not match, an `std::runtime_error` will be thrown. For example, if we give a 5-dimension vector `state` to our `HumanCarPlanner`, which expects a 6-dimension state vector:
```C++
try {
    Vector currentState = Vector {0, 0, 0, 1, 2}; // 5-dimension state vector
    Vector input = MyPlanner.update(currentState, humanInput, agents); // HumanCar expects 6-dimension state vector
    MyController.update(input);
}
catch (std::runtime_error e) {
    std::cout << e.what() << std::endl;
}
```
