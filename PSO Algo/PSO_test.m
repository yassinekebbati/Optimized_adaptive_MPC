%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author: Yassine Kebbati
% Date: 15/07/2021
% Optimization PSO_MPC_Autonomous_Driving
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


function out = PSO_test(problem, params,ref,vx,mu,wy)
    tic
    %%Problem Definition

    CostFunction = problem.CostFunction;  % Cost Function

    nVar = problem.nVar;     % Number of Unknown (Decision) Variables

    VarSize = [1 nVar];    %Matrix Size of Decision Variables

    AlphaMin = problem.AlphaMin;          %Lower Bound of Decision Variables
    AlphaMax = problem.AlphaMax;          %Upper Bound of Decision Variables
    NMin = problem.NMin;          %Lower Bound of Decision Variables
    NMax = problem.NMax;          %Upper Bound of Decision Variables
    NcMin = problem.NcMin;          %Lower Bound of Decision Variables
    NcMax = problem.NcMax;          %Upper Bound of Decision Variables
    NpMin = problem.NpMin;          %Lower Bound of Decision Variables
    NpMax = problem.NpMax; 
    Qy_Max = problem.Qy_Max;
    Qy_Min = problem.Qy_Min;
    R_Max = problem.R_Max;
    R_Min = problem.R_Min;
    VarMin = [AlphaMin NMin NcMin NpMin Qy_Min R_Min];
    VarMax = [AlphaMax NMax NcMax NpMax Qy_Max R_Max];
%     VarMin = [NMin NcMin NpMin Qy_Min R_Min];
%     VarMax = [NMax NcMax NpMax Qy_Max R_Max];

    %%Parameters of PSO


    MaxIt = params.MaxIt;     %Maximum Number of Iterations

    nPop = params.nPop;       %Population size (swarm size)

    w = params.w;           % inertia coefficient
    w_max = params.w_max;           % maximum inertia coefficient
    w_min = params.w_min;           % minimum inertia coefficient
    %wdamp = params.wdamp;     % Damping Ratio of Inertia Weight
    c1 = params.c1;          %Personal acceleration coefficient
    c2 = params.c2;          %social acceleration coefficient

    ShowIterationInfo = params.ShowIterInfo;    %the Flag for Showing interation information

    MaxVelocity = 0.2*(VarMax-VarMin);
    MinVelocity = -MaxVelocity; 
    

    %%Initialization

    % The Particle Template
    empty_particle.Position = [];
    empty_particle.Velocity = [];
    empty_particle.Cost = [];
    empty_particle.Best.Position = [];
    empty_particle.Best.Cost = [];


    %Create Population Array
    particle = repmat(empty_particle, nPop, 1);



    % Initialize Global Best
    GlobalBest.Cost = inf;



        %Initialize Population Members
        for  i = 1:nPop

            % Generate Random Solution
            a = unifrnd(AlphaMin,AlphaMax, 1); 
            Nc = randi([NcMin,NcMax],1); 
            Np= randi([2*Nc,NpMax],1);
            N = randi([NMin,round(Np/2)],1); 
            Qy = unifrnd(Qy_Min,Qy_Max, 1);
            R = unifrnd(R_Min,R_Max, 1);


            %particle(i).Position = [a N Nc Np Qy R];
            particle(i).Position = [a N Nc Np Qy R];


            % Initialize Velocity
            particle(i).Velocity = zeros(VarSize);

            % Evaluation
            particle(i).Cost = CostFunction(particle(i).Position,ref,vx,mu,wy);

            %Update the Personal Best
            particle(i).Best.Position = particle(i).Position;
            particle(i).Best.Cost = particle(i).Cost;

            %Update Global Best
            if particle(i).Best.Cost < GlobalBest.Cost
                GlobalBest = particle(i).Best;
            end


        end


        % Array to Hold Best Cost Value on Each Iteration
        BestCosts = zeros(MaxIt,1);

        


            %%Main Loop of PSO

            for it=1:MaxIt
                it

                for i=1:nPop

                    %Update Velocity
                    particle(i).Velocity = w*particle(i).Velocity...
                        + c1*rand(VarSize).*(particle(i).Best.Position - particle(i).Position)...
                        + c2*rand(VarSize).*(GlobalBest.Position - particle(i).Position);

                    % Apply Velocity Limits
                    particle(i).Velocity = max(particle(i).Velocity, MinVelocity);
                    particle(i).Velocity = min(particle(i).Velocity, MaxVelocity); 
                    %particle(i).Velocity

                    % Update Position
                    particle(i).Position = particle(i).Position + particle(i).Velocity;

                    %Apply Lower and Upper Bound Limits
                    particle(i).Position = max(particle(i).Position, VarMin);
                    particle(i).Position = min(particle(i).Position, VarMax);          
                    particle(i).Position(2:4) = round(particle(i).Position(2:4));
                    %check that N<Np
                    if  particle(i).Position(2) > particle(i).Position(4)
                        particle(i).Position(2) = particle(i).Position(4)
                    end
                    %check that Nc<Np
                    if  particle(i).Position(3) > particle(i).Position(4)
                        particle(i).Position(3) = particle(i).Position(4)
                    end

                    %particle(i).Position

                    % Evaluation
                    particle(i).Cost = CostFunction(particle(i).Position,ref,vx,mu,wy);

                    % Update Personal Besy
                    if particle(i).Cost < particle(i).Best.Cost

                        particle(i).Best.Position = particle(i).Position;
                        particle(i).Best.Cost = particle(i).Cost;

                        %Update Global Best
                        if particle(i).Best.Cost < GlobalBest.Cost
                            GlobalBest = particle(i).Best;
                        end

                    end

                end

                %Store the Best Cost Value
                BestCosts(it) = GlobalBest.Cost;

                %Display Iteration Information
                if ShowIterationInfo
                    disp(['Iteration ' num2str(it) ' : Best Cost = ' num2str(BestCosts(it))]);
                end


                %To be modified
                %acceleration Coeffecients

                if i <= round(0.2*MaxIt)
                    c1 = c1 + 0.05;
                    c2 = c2 - 0.05;
                elseif i <= round(0.35*MaxIt)
                    c1 = c1 + 0.02;
                    c2 = c2 - 0.02;
                elseif i <= round(0.75*MaxIt)
                    c1 = c1 - 0.035;
                    c2 = c2 + 0.035;
                else
                    c1 = c1 - 0.0015;
                    c2 = c2 + 0.0015;
                end

                %To be modified
                %Damping Inertia Coeffiecient
                %w = w * wdamp;%normal
                %w = w_max - it*(w_max-w_min)/MaxIt; %number 4
                %w = wdamp*(w_max - it*(w_max-w_min)/MaxIt); % number 1
                %w = w / (1+exp(w * wdamp)); % number 2
                %w = wdamp*(w / (1+exp(w_max - it*(w_max-w_min)/MaxIt))); % number 3
                w = w_min + exp((w_max - it*30*(w_max+w_min)/MaxIt))/3;


            out.pop = particle;
            out.BestSol = GlobalBest;
            out.BestCosts = BestCosts; 
            runtime = toc   

            end
end

