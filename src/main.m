clear all

%% Importing everything in the workspace
global_title = "slow kick motion";
addpath("./measurement/MotionCaptureData/")
addpath("./measurement/ForcePlatesData/")
addpath("./synchros/")
addpath("./functions/")

% Change the file path to match your data. But don't change the variable name.
data = fileread("high_jump.drf"); %Motion capture data
Tb = readtable("high_jump.csv"); %Force plate data
Tb_synchro = readtable("synchro_high_feet_jump_motion.csv"); %Data for synchronization

%% Extracting data
BSP_parameters;
data_extraction;
Force_plate_data_extract;

%% CoM and CoP models
CoM_CoP;

%% Check plots
robin_moving_plot; %uncomment this if you want to see Robin moving 
% phase_plot; %uncomment this to check if the phases (single/double support) are correctly extracted

%% Reaction forces model => from the motion to the foot wrenches
feet_forces_model;

%% Energy and power of the body
power_energy;

%% Plots
init_plots; % don't comment this
 feet_force_plots; % uncomment this if you want to see the feet force from the model vs from the measurements
% feet_moment_plots; % uncomment this if you want to see the feet moments from the model vs from the measurements
% CoP_plot;     % uncomment this if you want to see the CoP from the model vs from the measurements
% energy_power_plots;  % uncomment this if you want to see the enegy and power from the model
% feet_force_error_plots;
% Fz_plots;
% right_elbow_angle;