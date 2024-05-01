# Multi-robot Planning for Hetergoenous Robot Fleets in Dynamic Environments

Build code using ```. build.sh```. This requires a minimum g++ version of 10 to build correctly. If you are on Ubuntu 20.04 and earlier, you need to install g++10 manually, and use the command ```. build_ubuntu_20.sh``` to build the project. Make sure to use chmod to give execute permissions to the sh files before running them. 

Run code using ```. run.sh <solver_name> <map_name> <animation_speed> (optional to save animation)``` This requires python 3.9 or later, if you have a lower version of python, please use ```. run_python3.8.sh <solver_name> <map_name> <animation_speed>``` to run your code. However, the second script doesn't have the ability to save the animation. Again, make sure that you give execute permissions to the sh files first. 

The options that can be given to solver_name are _prioritized_ (for CPP), _cbs_ (for C2SC) and _ccbs_ (for C2SC+). For the map name option, you can give the name of any map file in the maps folder. Don't give the .txt extension with the map file name. The maps with the 0_ prefix don't have any movable obstacles or helper agents, and are purely for testing the vanilla CBS/PP implementations. The maps with 1_ prefix are the ones that were used to test the final code. 

Some examples commands are shown below:

Example to simply run and visualize: ```. run.sh cbs exp1_10```

Example to run and save animation: ```. run.sh prioritized exp1_10 1.0```

Example to run on an older Ubuntu version (20.04 or earlier) ```. run.sh ccbs exp1_14```