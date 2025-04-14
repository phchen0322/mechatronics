CONTENTS

This folder contains a copy of the offline model developed in the course.
model.m        : Matlab script that defines the necessary parameters and runs the simulation.
simulation.slx : Simulink file where the offline model is implemented.
OnlineModel    : folder containing a copy of the online model GUI used in the lab practices, including the chosen PID tuning values.
XYForce.xlsx   : Excel file where the damping polynomial fit is conducted.
rest of files  : data files storing the model parameters that model.m loads.


RUNNING THE SIMULATION

Open model.m with Matlab and run the script.

Simulation parameters, like initial position and velocity, target position, current and wind velocities and controller configurations can be modified in the "Simulation definition" section, between lines 7 and 16 of model.m. The Simulink model can be inspected by opening simulation.slx.