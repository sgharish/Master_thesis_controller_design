# Master_thesis_controller_design
@files: manipulatorsim.m, simlink_manipulator_MATLAB.slx, simlink_manipulator_UD.slx

@author: Harish Swaminathan Gopal

@softw: MATLAB, SIMULINK 

@toolbox: robotic system toolbox, control system toolbox


@brief:The adaptive backstepping controller is implemented for controlling the motion of the two link planar arm in a 3D space.
This work is for the thesis and it based on the controller design by 'Adaptive backstepping trajectory tracking control of robot manipulator' Qinglei Hun, Liang Xu, Aihua Zhang
Publiser: Journal of franklin institue.

@files: manipulatorsim.m:
"This is a .m file. In this file the model of the manipulator along with the path motion to generate the trajectory is defined. In addition this file can run the simulink file name 'simlink_manipulator_MATLAB.slx'"
simlink_manipulator_MATLAB.slx: 
"In this file the trajectory and controller of the model is implemented using the forward dynamics provided by MATLAB. The outputs of the simulink blocks are connected to the scope. The simulated path and error are send to the workspace of the matlab file 'manipulatorsim.m' so that it can be plotted and it will be easy to verify. "
manipulatorsim_UD.m:
"This file is similar to the 'manipulatorsim.m' file on minor difference is it can run 'simlink_manipulator_UD.slx'"
simlink_manipulator_UD.slx
"Similar to 'simlink_manipulator_MATLAB.slx' this file can generate trajectory and control the manipulator motion. The forward dynamics block of the 'simlink_manipulator_MATLAB.slx' is replaced by the user defined function block."

Run the files 'manipulatorsim.m' and 'manipulatorsim_UD.m' to see the desired path, simulated path and tracking error. The Lyapunov functions are available in the scopes of the simulink files. 
