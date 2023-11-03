%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author: Yassine Kebbati
% Date: 10/03/2021
% Control MPC-Autonomous_Driving
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


clear all, close all, clc

%laod simulation data
load DLane_change.mat;
load Params.mat;

%open model file
open MPC.slx;

%launch simulation
options = simset('SrcWorkspace','current');
S = sim('MPC',[],options);