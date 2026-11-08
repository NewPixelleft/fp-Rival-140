# RIVAL FINAL PROJECT
By NewPixelleft aka. Agwyn satria

the objectives are simpe *if* we don't implement any imrovizations. The guide to start this thing is simple indeed

## [1] Setup
Before starting the program, it would be wise to check for its dependencies. To check for dependencies, run
```bash
cd ~Documents/ros2-fp
bash scripts/install_dependencies.sh --check
```
If there exists any missing dependencies, run
```bash
cd ~Documents/ros2-fp
bash scripts/install_dependencies.sh --install
```

## [2] Build and Run
To quickly build and run this thing, we can do
```bash
chmod +x run.sh runbuild.sh teleop.sh
```
and then just type any of the bash files 
```bash
./runbuild.sh
```

## [3] The Controller CLI
once we had run `./runbuild.sh`, we would immediately be met with its controller CLI app. it'll open 5 terminals yes, but for a good reason.
```txt
- moveto <waypoint1> <waypoint2> ... <infWaypoint> 
    arg: --detect-aruco-logged [detect, and save aruco while logging it]
         --detect-aruco-non-logged [detect, but doesnt log it]
```