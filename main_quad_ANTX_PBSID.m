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
%% Experiment Definition for PBSID
% System definition:
% x = A*x + B*u
% y = C*x + D*u
% test 1
A_1ord = 0.9; B_1ord = 0.5; 
C_1ord = 0.5; D_1ord = 0.2;
% test 2
% A_1ord = -0.9; B_1ord = 0.5; 
% C_1ord = 0.5; D_1ord = 0.2;
% Order of the system
n = 1;
% Sampling Time
Ts_1ord = 0.01 ;
% Discrete time system definition 
sys_1ord = ss(A_1ord, B_1ord, C_1ord, D_1ord,Ts_1ord);
% Input Generation
A = 0.1 ; % Amplitude of signal
u3211_vect = u3211(A,Ts_1ord); % Function to generate u3211 input
u_1ord = u3211_vect(:,2); % u3211 input
t_1ord = u3211_vect(:,1); % Time for u3211 input
%t_1ord = 0:Ts:10; % Time for impulse input 
%u_1ord = 0.1*[zeros(100,1);ones(length(t1)-100,1)]; % Impulse input
% Output Generation + noise
y_1ord = lsim(sys_1ord, u_1ord, t_1ord); % Output simulaton
y_1ord = y_1ord + 0.01*rand(length(y_1ord),1); % Noise
%% P value definition for system
min_p = 2*n; max_p = 40; 
p = better_p(u_1ord,y_1ord,min_p,max_p,n,Ts_1ord,t_1ord);
clc
%% First Part of PBSID
[D_PBSID,S_PBSID,V_PBSID,Y,N] = pbsid_1part(u_1ord,y_1ord(:,1),p);
%n = input("looking the graph choose the order detected by the PBSID : ");
n = 1;
%% Second part of PBSID
[A_PBSID,B_PBSID,C_PBSID,K_PBSID] = pbsid_2part(D_PBSID,n,S_PBSID,V_PBSID,Y,N,p,u_1ord,y_1ord);
%% Verifications
% Eig error of matrix A
error_eig_A = eig(A_PBSID) - eig(A_1ord);
% Transfer funtion similarities in discrete
G_id = tf(ss(A_PBSID,B_PBSID,C_PBSID,D_PBSID,Ts_1ord))
G1 = tf(sys_1ord)
% Graphic comparison of outputs
figure ("Name","Graphic comparison of real outputs wrt identified sys")
plot(t_1ord,y_1ord,'k',t_1ord,lsim(ss(A_PBSID,B_PBSID,C_PBSID,D_PBSID,Ts_1ord), u_1ord, t_1ord),'r');
%% Structuring in order to find phisical parameters
% Definition of frequencies of interest
w_array = logspace(-3,3,1000); % To be tuned wrt the sys studied
% Transfer function in continuos
G_id = ss(A_PBSID,B_PBSID,C_PBSID,D_PBSID); % sys in continuous time
% Model identification
model_fun = @TF_model_1ord;
% Cost function, initial guess, upper and lower bounds detemination
fun = @(theta) sum(FDOE_problem(theta, w_array, G_id ,model_fun).^2);
theta0 = [A_1ord, B_1ord, C_1ord, D_1ord] + rand(1,4)*0.05; % First guess
lb = [-10, -10, -10, -10] ; % Lower bounds
ub = [10, 10, 10 ,10] ;   % Upper bounds
% Solving nonlinear least-squares problem
options = optimoptions('lsqnonlin','Display','iter','Algorithm','interior-point','FunctionTolerance',1e-6);
theta_opt = lsqnonlin(fun, theta0, lb, ub, options);
%% Verification of data
% Matricies reconstruction
A_id = theta_opt(1); B_id = theta_opt(2);
C_id = theta_opt(3); D_id = theta_opt(4);
% Error determination
errors_matricies = [norm(A_id-A_1ord);
                    norm(B_id-B_1ord);
                    norm(C_id-C_1ord);
                    norm(D_id-D_1ord)];
disp('Errors on the matricies identified wrt the actual matricies: ');
disp(errors_matricies);
%% Delete temporary files

if exist('slprj','dir')
    rmdir('slprj', 's')                                                    
end

%% END OF CODE