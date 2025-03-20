%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ANT-X SIMULATOR - MAIN                                                  %
% Authors:  Mattia Giurato (mattia.giurato@polimi.it)                     %
%           Paolo Gattazzo (paolo.gattazzo@polimi.it)                     %
% Date: 13/12/2017                                                        %
% Adapted to ANT-X 2DoF by:  Salvatore Meraglia (salvatore.meraglia@polimi.it)%
% Date: 22/12/2022                                                        %
%
% Further modified to include structure three-state identified longitudinal model
% 06/01/23 ML
% 
% Further modified to pick outputs with measurement error
% 03/01/24 ML
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% clearvars;
close all;
addpath('datasets','common','common/simulator-toolbox','common/simulator-toolbox/attitude_library','common/simulator-toolbox/trajectory_library');
addpath('functions')
clc;

%% Model parameters

% Initial model (state: longitudinal velocity, pitch rate, pitch angle; input: normalised pitching moment; outputs: state and longitudinal acceleration)

Xu=-0.1068;

Xq=0.1192;

Mu=-5.9755;

Mq=-2.6478;

Xd=-10.1647;

Md=450.71;

A=[Xu, Xq, -9.81; Mu, Mq, 0; 0, 1, 0];

B=[Xd; Md; 0];

C=[1, 0, 0; 0, 1, 0; 0, 0, 1; Xu, Xq, 0]; 

D=[0; 0; 0; Xd];

% Noise

%noise.Enabler = 0;
noise.Enabler = 1;

noise.pos_stand_dev = noise.Enabler * 0.0011;                            	%[m]

noise.vel_stand_dev = noise.Enabler * 0.01;                               %[m/s]

noise.attitude_stand_dev = noise.Enabler * deg2rad(0.33);                 %[rad]
noise.ang_rate_stand_dev = noise.Enabler * deg2rad(1);                   %[rad/s]

% Delays

delay.position_filter = 1;
delay.attitude_filter = 1;
delay.mixer = 1;

%% Load controller parameters

parameters_controller                    

%% M injection example (sweeep: first column time vector, secondo column time history of pitching moment) 

load ExcitationM

SetPoint=[0,0];

%% Values selected

t=ExcitationM(:,1);

simulation_time=t(end)-t(1);

%% Launch SIMULATOR

%sim Simulator_Single_Axis
simulation_data = sim('Simulator_Single_Axis','SrcWorkspace', 'current');

%% Task 1
%% Plot of the input
figure('Name','Input plot')
u_time=ExcitationM(:,1);
u_value=ExcitationM(:,2);
u_value=u_value(u_time>26 & u_time<100);
u_time=u_time(u_time>26 & u_time<100);
plot(u_time, u_value)
grid
title('Input Signal')
ylabel('u'); ylim([-0.11, 0.11]); xlabel('time [s]')
%% Model ID
% Time span for the model identification
time = 0:sample_time:simulation_time;
% Model call
model_fun = 'drone_model';
% Real Parameter Vector
real_parameters = [Xu; Xq; Mu; Mq; Xd; Md];
% Function call
[identification varargout] = Model_identification(simulation_data,sample_time,model_fun,real_parameters);
%% Task 2
%% PBSID

% Definizione del sistema discreto
A1 = 0.5;
B1 = 0.5;
C1 = 1;
D1 = 0.2;
Ts = 0.01;  % Tempo di campionamento
sys1 = ss(A1, B1, C1, D1, Ts);
% Simulazione dell'input e dell'output
t1 = 0:Ts:10;  % Tempo di simulazione
u1 = [zeros(100,1);ones(length(t1)-100,1)];  % Ingresso casuale per eccitare il sistema
y1 = lsim(sys1, u1, t1); % Output simulaton
y1 = y1 + 0.01*rand(length(y1),1);
% % Evaluation of the method by Step Function
% A1 = -2;
% B1 = 1;
% C1 = 1;
% D1 = 0;
% Ts = 0.001;
% %sys1 = c2d(ss(A1,B1,C1,D1),Ts,'tustin');
% sys1 = ss(A1,B1,C1,D1);
% t1 = 0 : Ts : 10;
%u1 = sin(t1)';
% u1 = ones(length(t1),1);
% [y1,t_out1,x1] = lsim(sys1,u1,t1);
% y1 = y1 + 0.01*rand(length(t1),1);
% Our Sys
% u1 = u_value(u_time>26 & u_time<100);
% y1 = simulation_data.q(u_time>26 & u_time<100);
% p = 100 ;
%%
u1 = (u1 - mean(u1)) / std(u1);
y1 = (y1 - mean(y1)) / std(y1);
p= 12;
[D_id,S_svd,V_svd,Y,N] = pbsid12(u1,y1(:,1),p);
%%
n = 1;
[A_id,B_id,C_id,K_id] = pbsid2(D_id,n,S_svd,V_svd,Y,N,p,u1,y1);
eig(A_id)
eig(A1)
sys_id = ss(A_id,B_id,C_id,D_id,Ts);
tf(sys_id)
tf(sys1)
model_fun = 'model_test';
%eig(A1)
%[identification] = structuring1(A_id,B_id,C_id,D_id,model_fun,u1,t1,Ts);
%%
[S,X] = dordvarx(u1,y1,p,p,'tikh','gcv');
figure, semilogy(S,'*');
%% Set system order
order = 1;

x = dmodx(X,order);
[Ai,Bi,Ci,Di,Ki] = dx2abcdk(x,u1,y1,p,p);
eig(Ai)
%% Delete temporary files

if exist('slprj','dir')
    rmdir('slprj', 's')                                                    
end

%% END OF CODE