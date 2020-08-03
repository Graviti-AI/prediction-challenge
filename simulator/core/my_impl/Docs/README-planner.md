# Documentation for custom class derived from `Planner`

## Header files

Please include `Planners/Planner.hpp` before you derive your custome class from `Planner`. 
```C++
#include "Planners/Planner.hpp"
class MyPlanner : public Planner {
    // definition body ...
}
```

## Construction

You are supposed to specify the dimension of `state` vector and `input` vector in the argument list of the construction of the base class `Planner`. 
For example, if we want to derive a `MyPlanner` class, we should write the construction as follows:
```C++
MyPlanner::MyPlanner(/* your argument list here */) : Planner(6, d) {
    // construction body ...
}

```
since a `HumanCar` has a state vector of dimension 6 (location X, location Y, yaw angle, speed X, speed Y, angular speed of yaw), and an input vector of dimension d (the `input` vector is the output of Zeji's planner).

## Overriding the virtual method

All the interface of a `Planner` lies in the virtual method `update`. The signature of this function is:
```C++
typedef std::vector<double> Vector;
Vector Planner::update(Vector currentState, const Vector &humanInput, std::vector<Agent*> agents);
```

It takes input of the current state of the agent, the human input (if your planner needs it), and information of other agents. 
It will have the fourth argument `Environment` in future versions, but right now we do not have this parameter.

It outputs the input vector, which the simulator will pass to the agent controller and use it to update the agent state for the next iteration.

For example, for the HumanCar, we should write our overridden implementation as:
```C++
Vector HumanCarPlanner::update(Vector currentState, const Vector &humanInput, std::vector<Agent*> agents) {
    return Vector(humanInput); // directly use the human input for the planner output
}
```

## If you need information of other agents...

For each agent, there is a `getState()` method, which gives the state vector of this agent.
For a car, this state vector has 6 dimensions: location x, location y, yaw angle, speed x, speed y, angular velocity of yaw.
You can use the vector `agents` to get information of the environment.

## Exception handling

If the dimensions of the vectors do not match, an `std::runtime_error` will be thrown. For example, if we give a 5-dimension vector `state` to our `HumanCarPlanner`, which expects a 6-dimension state vector:
```C++
try {
    Vector currentState = Vector {0, 0, 0, 1, 2}; // 5-dimension state vector
    humanCarPlanner.update(currentState, humanInput, agents); // HumanCar expects 6-dimension state vector
}
catch (std::runtime_error e) {
    std::cout << e.what() << std::endl;
}
```
