%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author: Yassine Kebbati
% Date: 15/07/2021
% Optimization PSO_MPC_Autonomous_Driving
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function J = mpc_param(parms,ref,vx,mu,wy);



dt=0.1;
N=18/0.1;  

set_param('MPC/a','Value','parms(1)');
set_param('MPC/N','Value','parms(2)');
set_param('MPC/Nc','Value','parms(3)');
set_param('MPC/Np','Value','parms(4)');
set_param('MPC/Qy','Value','parms(5)');
set_param('MPC/R','Value','parms(6)');
set_param('MPC/ref','Value','ref');
set_param('MPC/vx','Value','vx');
set_param('MPC/mu','Value','mu');
set_param('MPC/wy','Value','wy');
% set_param('MPC1/pos','Value','pos');
%load('Dlanechange6s.mat')
load('data.mat')
options = simset('SrcWorkspace','current');
S = sim('MPC',[],options);

% t = Data.Time;
% e = Data.Data(:,1);
% r = Data.Data(:,2);
% y = Data.Data(:,3);
% u = Data.Data(:,4);

t = S.Data.Time;
r = S.Data.Data(:,1);
y = S.Data.Data(:,2);
e = S.Data.Data(:,3);
u = S.Data.Data(:,4);
yp = S.Data.Data(:,5);
up = S.Data.Data(:,6);

% assignin('base','u1',sum(u(:))/length(u));
% assignin('base','r1',r(end));
% assignin('base','e1',sum(e(:))/length(e));

assignin('base','u',u);
assignin('base','r',r);
assignin('base','e',e);
assignin('base','y',y);
assignin('base','yp',yp);
assignin('base','up',up);
    
xlswrite('anfis2',{'U(k)' 'U(k-1)' 'Y(k)' 'Y(k-1)' 'R' 'E'},1,'A1')
xlswrite('anfis2',[u,up,y,yp,r,e],1,'A2')
   
% %%%mean squared error
J = 1/N*sum((r(:)-y(:)).^2);

% integral absolute error
%J = sum(abs(r(:)-y(:)))

% integral squared error
%J = sum(abs(r(:)-y(:)).^2)


% integral absolute time error
%J = sum(t.*abs(r(:)-y(:))) 


%minimize error and control effort
% E = 1/N*sum((r(:)-y(:)).^2)  
% U = 1/N*sum(u(:).^2)
% J = E + U