cmake_minimum_required(VERSION 3.10)
project(simulator)

SET(CMAKE_CXX_FLAGS -pthread)
SET(CMAKE_EXE_LINKER_FLAGS " -no-pie")
set(CMAKE_CXX_STANDARD 14)
#set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)


find_package(protobuf CONFIG REQUIRED)
find_package(gRPC CONFIG REQUIRED)
find_package(Threads REQUIRED)
find_package(OpenMP REQUIRED)


add_subdirectory(core/my_impl/Lanelet2Lib)
add_subdirectory(core/my_impl/rpclib)

include_directories(
    .
)

include_directories(core/my_impl/FourWheel/include)
include_directories(core/my_impl/FourWheel/include/FourWheelModel)
include_directories(core/my_impl/PidControllerLib/include)
include_directories(core/my_impl/PidControllerLib/include/PidControllerLib)
include_directories(core/my_impl/Behaviours)
include_directories(core/my_impl/jsoncpp/include)
include_directories(core/my_impl/rpclib/include)
include_directories(
    ${LANELET2_INCLS_DIRS}
)

file(GLOB_RECURSE FourWheel_INCL_FILES "core/my_impl/FourWheel/include/FourWheelModel/*.h")
file(GLOB_RECURSE FourWheel_SOURCE_FILES "core/my_impl/FourWheel/src/*.cpp")
file(GLOB_RECURSE ALGLIB_FILES "core/my_impl/alglib/*")
file(GLOB_RECURSE EBPLANNER_INCL_FILES "core/my_impl/PlannerLib/EBplanner/*.h")
file(GLOB_RECURSE EBPLANNER_SOURCE_FILES "core/my_impl/PlannerLib/EBplanner/*.cpp")
file(GLOB_RECURSE PLANNERLIB_INCL_FILES "core/my_impl/PlannerLib/*.h" "core/my_impl/PlannerLib/*.hh" "core/my_impl/PlannerLib/*.hpp")
file(GLOB_RECURSE PLANNERLIB_SOURCE_FILES "core/my_impl/PlannerLib/*.cpp" "core/my_impl/PlannerLib/*.cc")



add_executable(simulator 
    main.cpp 
    service/impl/service_impl.cpp 
    service/proto/simulator.grpc.pb.cc 
    service/proto/simulator.pb.cc 
    service/service.cpp

    core/trajectory.hpp
    core/simulator.hpp
    core/simulator.cpp

    core/my_impl/Server/Server.hpp
    core/my_impl/Server/Server.cpp
    core/my_impl/Server/ServerRespondInput.cpp
    core/my_impl/Server/ServerRespondManager.cpp
	core/my_impl/Server/ServerRespondRendering.cpp

    core/my_impl/UDPserver/CommunicationTask.cpp
    core/my_impl/UDPserver/CommunicationTask.h
    core/my_impl/UDPserver/MyClientPool.cpp
    core/my_impl/UDPserver/MyClientPool.h

    core/my_impl/my_simulator_impl.hpp
    core/my_impl/my_simulator_impl.cpp  

    core/my_impl/Simulator/Simulator.hpp
    core/my_impl/Simulator/Simulator.cpp
    core/my_impl/Simulator/CollisionOrNot.h
    core/my_impl/Simulator/CollisionOrNot.cpp

    core/my_impl/threadPool/MyThreadPool.cpp
    core/my_impl/threadPool/MyThreadPool.hpp
    core/my_impl/threadPool/BusyThreadContainer.cpp

    core/my_impl/threadPool/BusyThreadContainer.hpp
    core/my_impl/threadPool/IdleThreadContainer.cpp
    core/my_impl/threadPool/IdleThreadContainer.hpp
    core/my_impl/threadPool/MyThread.cpp
    core/my_impl/threadPool/MyThread.hpp
    core/my_impl/threadPool/Task.cpp
    core/my_impl/threadPool/Task.hpp
    core/my_impl/threadPool/TaskContainer.cpp
    core/my_impl/threadPool/TaskContainer.hpp

    core/my_impl/Maps/MapInfo.cpp
    core/my_impl/Maps/MapInfo.hpp
    core/my_impl/Maps/TrafficInfo.cpp 
    core/my_impl/Maps/TrafficInfo.hpp
    core/my_impl/Maps/LaneletMapReader.hpp
    core/my_impl/Maps/LaneletMapReader.cpp 

    core/my_impl/Agents/Agent.hpp 
    core/my_impl/Agents/HumanCar.hpp
    core/my_impl/Agents/RealCar.hpp
    core/my_impl/Agents/VirtualCar.hpp
    core/my_impl/Agents/Agent.cpp 
    core/my_impl/Agents/HumanCar.cpp
    core/my_impl/Agents/RealCar.cpp
    core/my_impl/Agents/VirtualCar.cpp
    core/my_impl/Agents/PedestrianAgent.cpp
    core/my_impl/Agents/PedestrianAgent.hpp        
    core/my_impl/Agents/BehaveCar.hpp
    core/my_impl/Agents/BehaveCar.cpp
    core/my_impl/Agents/NewVirtualCar.hpp
    core/my_impl/Agents/NewVirtualCar.cpp

    core/my_impl/Behaviours/Behaviour.hpp
    core/my_impl/Behaviours/AoBehaviour.hpp
    core/my_impl/Behaviours/LaneletBehaviour.hpp
    core/my_impl/Behaviours/CarGenerator.hpp
    core/my_impl/Behaviours/pure_pursuit.hpp
    core/my_impl/Behaviours/utils.hpp
    core/my_impl/Behaviours/ReplayGenerator.hpp 
    core/my_impl/Behaviours/FSM.hpp

    core/my_impl/Planners/Planner.hpp
    core/my_impl/Planners/HumanCarPlanner.hpp
    core/my_impl/Planners/RealCarPlanner.hpp
    core/my_impl/Planners/VirtualCarPlanner.hpp
    core/my_impl/Planners/TrivialPlanner.hpp
    core/my_impl/Planners/PedestrianPlanner.hpp 
    core/my_impl/Planners/HumanPedestrianPlanner.hpp
    core/my_impl/Planners/AstarPlanner.hpp
    core/my_impl/Planners/CILQRPlanner.hpp
    core/my_impl/Planners/EBPlanner.hpp

    core/my_impl/Controllers/Controller.cpp
    core/my_impl/Controllers/Controller.hpp
    core/my_impl/Controllers/HumanCarController.cpp
    core/my_impl/Controllers/HumanCarController.hpp
    core/my_impl/Controllers/RealCarController.cpp
    core/my_impl/Controllers/RealCarController.hpp
    core/my_impl/Controllers/VirtualCarController.cpp
    core/my_impl/Controllers/VirtualCarController.hpp
    core/my_impl/Controllers/SimplePidController.cpp
    core/my_impl/Controllers/SimplePidController.hpp
    core/my_impl/Controllers/FourWheelController.cpp
    core/my_impl/Controllers/FourWheelController.hpp
    core/my_impl/Controllers/PedestrianController.cpp
    core/my_impl/Controllers/PedestrianController.hpp
    core/my_impl/Controllers/BicyclePurePursuit.cpp
    core/my_impl/Controllers/BicyclePurePursuit.hpp
    core/my_impl/PidControllerLib/include/PidControllerLib/PidController.h
    core/my_impl/PidControllerLib/src/PidController.cpp

    core/my_impl/Models/Model.cpp 
    core/my_impl/Models/Model.hpp  
    core/my_impl/Models/FourWheelModel.cpp  
    core/my_impl/Models/FourWheelModel.hpp  
    core/my_impl/Models/HumanCarModel.cpp  
    core/my_impl/Models/HumanCarModel.hpp  
    core/my_impl/Models/RealCarModel.cpp  
    core/my_impl/Models/RealCarModel.hpp  
    core/my_impl/Models/VirtualCarModel.cpp 
    core/my_impl/Models/VirtualCarModel.hpp
    core/my_impl/Models/PedestrianModel.cpp 
    core/my_impl/Models/PedestrianModel.hpp

    core/my_impl/Predictors/Predictor.hpp
    core/my_impl/Predictors/Predictor.cpp 
    core/my_impl/Predictors/ConstantSpeedPredictor.hpp
    core/my_impl/Predictors/ConstantSpeedPredictor.cpp
    core/my_impl/Predictors/PyPredictor.hpp
    core/my_impl/Predictors/PyPredictor.cpp

    ${FourWheel_INCL_FILES}
    ${FourWheel_SOURCE_FILES}
    ${ALGLIB_FILES}
    ${EBPLANNER_INCL_FILES}
    ${PLANNERLIB_INCL_FILES}
)

find_library(sim_lib
    sim
    ${CMAKE_CURRENT_SOURCE_DIR}
    NO_DEFAULT_PATH)

find_library(jsoncpp_lib
    jsoncpp
    ${CMAKE_CURRENT_SOURCE_DIR}
    NO_DEFAULT_PATH)

target_link_libraries(simulator
    rpc
    gRPC::grpc++
    lanelet2
    OpenMP::OpenMP_CXX
    ${sim_lib}
    ${jsoncpp_lib}
)

