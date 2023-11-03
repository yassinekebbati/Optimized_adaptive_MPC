
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author: Yassine Kebbati
% Date: 15/07/2021
% Optimization PSO_MPC_Autonomous_Driving
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


clc;clear;close all;



%%Problem Definition

problem.CostFunction = @(x) Sphere(x);  % Cost Function
problem.nVar = 10;     % Number of Unknown (Decision) Variables
problem.VarMin = -10;          %Lower Bound of Decision Variables
problem.VarMax = 10;          %Upper Bound of Decision Variables


%%Parameters of PSO


params.MaxIt = 1000;     %Maximum Number of Iterations
params.nPop = 50;       %Population size (swarm size)
params.w = 1;           % inertia coefficient
params.wdamp = 0.99     % Damping Ratio of Inertia Weight
params.c1 = 2;          %Personal acceleration coefficient
params.c2 = 2;          %social acceleration coefficient
params.ShowIterInfo = true;  %Flag for showing Iteration







%% Calling PSO


out = PSO(problem, params);

BestSol = out.BestSol;
BestCosts = out.BestCosts;



%% Results

figure;
%plot(BestCosts, 'LineWidth',2);
semilogy(BestCosts, 'LineWidth',2);
xlabel('Iteration');
ylabel('Best Cost');
grid on;