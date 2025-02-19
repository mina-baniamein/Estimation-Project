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

% %% Launch SIMULATOR
% 
% sim Simulator_Single_Axis

%% Task 1 
time = 0:sample_time:simulation_time;
%% plot input

figure
u_time=ExcitationM(:,1);
u_value=ExcitationM(:,2);
u_value=u_value(u_time>26 & u_time<100);
u_time=u_time(u_time>26 & u_time<100);
plot(u_time, u_value)
grid
title('input signal')
ylabel('u')
xlabel('time [s]')
%% 

odefun= @drone_model;

load_system('Simulator_Single_Axis');

simulation = sim('Simulator_Single_Axis','SrcWorkspace', 'current');


data.ax = simulation.ax;
data.q = simulation.q;
input = simulation.Mtot;

output = [data.q data.ax]; % Measured acceleration and pitch rate

sim_data = iddata(output, input, sample_time);
data_fd = fft(sim_data); % output of the simulation in the frequency domain


% Initial guess for the parameters
initial_guess = [0, 0, 0, 0, 0, 0]; % Replace with your initial guesses if known

% Define the parameters as a cell array
parameters = {initial_guess(1), initial_guess(2), initial_guess(3), ...
              initial_guess(4), initial_guess(5), initial_guess(6)};

sys_init = idgrey(odefun, 'c', parameters);

% Model Identification
identification = struct;
estimated_model = greyest(data_fd, sys_init);
identification.parameters = estimated_model.Report.Parameters.ParVector;
identification.fit = estimated_model.Report.Fit.FitPercent;
identification.covariance = getcov(estimated_model);
identification.matrix={estimated_model.A; estimated_model.B; estimated_model.C; estimated_model.D};
identification.estimated_model=estimated_model;


%% Delete temporary files

if exist('slprj','dir')
    rmdir('slprj', 's')                                                    
end

%% END OF CODE
%Functions
function [A,B,C,D] = drone_model(Xu,Xq,Mu,Mq,Xd,Md)

A=[Xu, Xq, -9.81; Mu, Mq, 0; 0, 1, 0];

B=[Xd; Md; 0];

C=[1, 0, 0; 0, 1, 0; 0, 0, 1; Xu, Xq, 0]; 

D=[0; 0; 0; Xd];

end




