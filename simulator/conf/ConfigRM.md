# README (config)

Each config file contains the following entries:

1. **Map**: Name of the Map you want to run the simulation.

2. **Track**: The track file ID corresponding to this simulation.

   - The simulator will use the CSV file located at `CSV/Map/vehicle_tracks_MAP_Track.csv`, e.g. `CSV/DR_CHN_Merging_ZS/vehicle_trakcs_000.csv`.

3. **StartTimestamp(ms)** : The start timestamp 

4. **EndTimestamp(ms)**: The end timestamp

5. **RobotCarNum**: the Number of Robot Car you want to add in the simulation.

   - The next  `RobotCarNum + 1` rows describe the initial states of each robot car. **We support two formats.**

   - **Format 1**

     ```
     InitState:track_id,Planner,Planner.Para,in_Predictor,in_Predictor.dt,in_Predictor.horizon,ex_Predictor,ex_Predictor.dt,ex_Predictor.horizon,ego_car
     
     1. track_id: ID of the robot car; 
     2. Planner,Planner.Para: Planner type and Planner parameter (A:Aggressive;N:Neutral;D:Defensive) of the robot car;
     3. in_Predictor,in_Predictor.dt,in_Predictor.horizon: Interal Predictor type; Time step of Interal Predictor; Prediction horizon of Interal Predictor;
     4. ex_Predictor,ex_Predictor.dt,ex_Predictor.horizon: Flag for whether use Exteral Predictor; Time step of Exteral Predictor; Prediction horizon of Exteral Predictor;
     5. ego_car: Flag for whether this car is the ego vehicle of this simulation.
     
     e.g.
     13 Astar N GroundTruth 0.1 3.0 no 0.1 3.0 yes
     
     If you choose format 1, the initial states of the robot car will be exactly the same as the states with which this car first appears at the CSV file.
     ```

   - **Format 2**

     ```
     InitState:track_id,start_ts,x,y,yaw,v_lon,v_lat,v_yaw,length,width,start_lanelet_ID,end_lanelet_ID,Planner,Planner.Para,in_Predictor,in_Predictor.dt,in_Predictor.horizon,ex_Predictor,ex_Predictor.dt,ex_Predictor.horizon,ego_car
     
     Format 2 allow users tp specify the initial states.
     
     Additional terms:
     1. start_ts (ms): At which which timestep you want to add this car in;
     2. x,y,yaw,v_lon,v_lat,v_yaw:  Init postion and velocity of the car;
     3. length,width: Length and width of the robot car;
     4. start_lanelet_ID,end_lanelet_ID: start lanelet ID and end lanelet ID of the routing for this car
     
     e.g.
     15 45200 3.253713 6.686023 3.117403 4.349906 0.00 0.00 4.430000 1.750000 30000 30008 Astar N GroundTruth 0.1 3.0 no 0.1 3.0 yes
     ```

6. **ReplayCarWithPredictor**: Num of Reply Car you want to sign a Predictor for it.

   - The next `ReplayCarWithPredictor + 1` rows describe the initial states of each replay car.

     ```
     InitState:track_id,in_Predictor,in_Predictor.dt,in_Predictor.horizon,ex_Predictor,ex_Predictor.dt,ex_Predictor.horizon
     
     1. track_id: ID of the replay car; 
     2. in_Predictor,in_Predictor.dt,in_Predictor.horizon: Interal Predictor type; Time step of Interal Predictor; Prediction horizon of Interal Predictor;
     3. ex_Predictor,ex_Predictor.dt,ex_Predictor.horizon: Flag for whether use Exteral Predictor; Time step of Exteral Predictor; Prediction horizon of Exteral Predictor;
     ```

7. **EgoEndPosition**: The original position for ego vehicle at the original end time in track. (use for grading)

8. **TargetRightofWay**: Vehicles who own the right of way other than ego vehicle. (use for grading)

# Range of InitState
1. **v_lon**: 0~9 m/s

2. **v_lat**: 0 (We have this term. But not used for now.)

3. **v_yaw**: 0 (We have this term. But not used for now.)

4. **x,y,yaw**: You should set it inside one of the Lanelet in Start Lanelet id list.

5. **start_lanelet_ID**: The Start Lanelet id lists are following.
>**"DR_CHN_Roundabout_LN"**:30093 30090 30084 30060 30003 30006
>**"DR_DEU_Merging_MT"**:30003 30000 
>**"DR_DEU_Roundabout_OF"**:30006 30029 30031
>**"DR_USA_Intersection_EP0"**:30057 30056 30048 30022 30021 30019 30027 30032 
>**"DR_USA_Intersection_EP1"**:30071 30069 30068 30067 30066 30065 30049 30013 30032 30033 30042
>**"DR_USA_Intersection_GL"**:30081 30069 30066 30058 30057 30047 30017 30000 30039
>**"DR_USA_Intersection_MA"**:30021 30018 30015 30014 30029 30035 30040 30046
>**"DR_USA_Roundabout_EP"**:30058 30056 30053 30015 30003 30001 30007 30030 30034
>**"DR_USA_Roundabout_FT"**:30021 30013 30011 30006 30023 30025 30046
>**"DR_USA_Roundabout_SR"**: 30007 30002 30014 30019

6. **end_lanelet_ID**: The End Lanelet id lists are following.
>**"DR_CHN_Roundabout_LN"**:30088 30016 30002 30000 30001 30007 30044 
>**"DR_DEU_Merging_MT"**:30008
>**"DR_DEU_Roundabout_OF"**:30022 30028 30037
>**"DR_USA_Intersection_EP0"**:30058 30055 30047 30018 30016 30023 30029
>**"DR_USA_Intersection_EP1"**:30075 30074 30073 30072 30070 30063 30020 30037 30044 30046
>**"DR_USA_Intersection_GL"**:30077 30053 30001 30009 30024 30026 30029 30030
>**"DR_USA_Intersection_MA"**:30065 30060 30059 30053 30022 30036 30045
>**"DR_USA_Roundabout_EP"**:30057 30054 30009 30037 30042 30043
>**"DR_USA_Roundabout_FT"**:30047 30017 30012 30005 30007 30010
>**"DR_USA_Roundabout_SR"**:30000 30003 30004 30021
