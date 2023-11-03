%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author: Yassine Kebbati
% Date: 15/07/2021
% Optimization PSO_MPC_Autonomous_Driving
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%clc;clear;close all;
open MPC.slx

% tic
%%Problem Definition
problem.CostFunction = @(x,ref,vx,mu,wy) mpc_param(x,ref,vx,mu,wy);  % Cost Function
%problem.CostFunction = @(x) Sphere(x);  % Cost Function
problem.nVar = 6;           % Number of Unknown (Decision) Variables
problem.AlphaMin = 0.75;    %exp(-3);          %Lower Bound of Decision Variables
problem.AlphaMax = 0.99;    %1-exp(-3);          %Upper Bound of Decision Variables
problem.NpMin = 1;          %Lower Bound of Decision Variables
problem.NpMax = 55;         %Upper Bound of Decision Variables
problem.NMin = 1;           %Lower Bound of Decision Variables
problem.NMax = 9;           %NpMax %Upper Bound of Decision Variables
problem.NcMin = 1;          %Lower Bound of Decision Variables
problem.NcMax = 18;         %Upper Bound of Decision Variables
problem.Qy_Max = 10;        %Upper Bound of Qy
problem.Qy_Min = 0.1;       %Lower Bound of Qy
problem.R_Max = 0.1;        %Upper bound of R
problem.R_Min = 0.001;      %Lower Bound of R






%%Parameters of PSO


params.MaxIt = 5;     %Maximum Number of Iterations
params.nPop = 10;       %Population size (swarm size)
params.w = 1;           % inertia coefficient
params.w_max = 0.9;           % maximum inertia coefficient
params.w_min = 0.4;           % minimum coefficient
%params.wdamp = 0.99;     % Damping Ratio of Inertia Weight
params.c1 = 2;          %Personal acceleration coefficient
params.c2 = 2;          %social acceleration coefficient
params.ShowIterInfo = true;  %Flag for showing Iteration
 



k=0;

for i=-10:4:14  %-15:2:+15
    ref=i;
    for j=5:2:25   %3:3:27
        vx=j;
        for l=-30:6:30          %-70:14:70
            wy=l;
            for m=0.5:0.1:0.9      %0.45:0.1:0.95
                mu=m;
                k= k+1;
%% Calling PSO
        %ref = -11;Vx=3;

                out = PSO_test(problem,params,ref,vx,mu,wy);

                BestSol = out.BestSol;
                BestCosts = out.BestCosts;


                a(k,1) = out.BestSol.Position(1);
                N(k,1) = out.BestSol.Position(2);
                Nc(k,1) = out.BestSol.Position(3);
                Np(k,1) = out.BestSol.Position(4);
                Qy(k,1) = out.BestSol.Position(5);
                R(k,1) = out.BestSol.Position(6);
                Ref(k,1) = ref;
                Vx(k,1) = vx;
                Mu(k,1) = mu;
                Wy(k,1)  = wy;



                xlswrite('MPC_DATA',{'reference' 'Vx' 'Mu' 'Wy' 'a' 'N' 'Nc' 'Np' 'Qy' 'R'},1,'A1')
                xlswrite('MPC_DATA',[Ref,Vx,Mu,Wy,a,N,Nc,Np,Qy,R],1,'A2')

        %% Results

%         figure;
%         %plot(BestCosts, 'LineWidth',2);
%         semilogy(BestCosts, 'LineWidth',2);
%         xlabel('Iteration');
%         ylabel('Best Cost');
%         title('Improved PSO algorithm learning');
%         grid on;
%         toc
            end
        end
    end
end
