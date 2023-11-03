%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author: Yassine Kebbati
% Date: 15/07/2021
% Optimization PSO_MPC_Autonomous_Driving
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%




clc;clear;close all;



%%Problem Definition

problem.CostFunction = @(x) Sphere(x);  % Cost Function
problem.nVar = 5;     % Number of Unknown (Decision) Variables
problem.VarMin = -10;          %Lower Bound of Decision Variables
problem.VarMax = 10;          %Upper Bound of Decision Variables


%%Parameters of PSO

kappa = 1;
phi1 = 2.05;
phi2 = 2.05;
phi = phi1 + phi2
chi = 2*kappa/abs(2-phi-sqrt(phi^2-4*phi));

params.MaxIt = 100;     %Maximum Number of Iterations
params.nPop = 10;       %Population size (swarm size)
params.w = chi;           % inertia coefficient
params.wdamp = 1     % Damping Ratio of Inertia Weight
params.c1 = chi*phi1;          %Personal acceleration coefficient
params.c2 = chi*phi2;          %social acceleration coefficient
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