# Documentation for the whole program

## How to use

Open the Sever(first):

    open the agentSim(home/CLionProject/framework_jin)

Virtual Car(home/mkz-mpc-control/output.txt) (optional):

    Each row: current car number & first car id & first car x & first car y & first car angle & first car vx first car vy & first car angular v & second ….
    0.1 s update one row.
    If fisrt three col is 0 (x,y,angle) —> car disappear.
    
    To add the virtual car, click "Startvc" at the manager website.
    To start it, click "Start" at the manager website.(If you have started it once, you don't need to do this again.)


Human Car(optional):

    (1) Initial steel, brake, accelerator : Desktop: FFB_Cs ——> initDevice ——> start
    (2) Send car info(steel, brake, accelerator) to server: Desktop/Fan/MscArch/UDP_project/UDP_proecjt.sln ——> run
    (3) Rendering（渲染）: D:/sim/11.13/Rending(network)/launch.ps1
    (4) Chose the agenttype as FourWheelCar and type in the initial states, click "add" at the manager website.
    (5) To start it, click "Start" at the manager website.(If you have started it once, you don't need to do this again.)
    
    How to create a new map (rendering) : D:/sim/11.13/12.13/MyProejct/MyProject.uproject
        Change IP address: open .sln (need to rebuild solution and new to create a new rendering)
        Unreal4 (how to create road in unreal4 )
        after modify, create a new rendering: File -> Package Project -> windows -> 64bit ———> create a new folder called windowNoEditor
        Copy all file under windowNoEditor to D:/sim/11.13/Rendering(network)/ front, left, right (replace)
        How to connect with server: MyProject.sln/ Game

Real Car(optional):

    (1) Collect data from real car: home/mkz-mpc-control ——> open power shell ——> source devel/setup.sh ———> rosrun collect collect_node (For real car)  and then use manager addRealCar
    (2) Chose the agenttype as RealCar and type in the initial states, click "add" at the manager website.
    (3) To start it, click "Start" at the manager website.(If you have started it once, you don't need to do this again.)

## How the Main thread Work

When you run the server, it will creat a thread pool include 10 threads for simulator to calculate agents.
The thread pool will open one extra thread to manage these 10 threads automatically.

Then, it will creat a simulator and a server.
The simulator and the server will share their SimulatorState, AgentDictionary and humanInputs, so that the manager can manage the simulator. And, other session can get what they need.
The server will sign three kinds of services for for input, manager and rendering and open 5 threads to waiting for the command.

At last, it will invoke simulator::run which is a endless loop to update the simulator.


## Documents

If you change some documents and want to regenerate the document. Remember to change the address in the Doxyfile.

