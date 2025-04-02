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

clearvars;
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
% Selection on the input

x = 0;
while (x ~= 1) && (x ~= 2)
    x = input('Select 1 for ExitationM and 2 for input u3211 : ');
    if x == 1
        load ExcitationM
    elseif x == 2
        Ts_3ord = 0.001; % Sampling time for u3211
        ExcitationM = u3211(0.1,Ts_3ord); % Function to generate u3211 input
        ExcitationM = [ExcitationM((1:end-1),:); u3211(0.1,Ts_3ord)]; % Function to generate u3211 input
        ExcitationM(:,1) = (0:Ts_3ord:16)';
    else
        disp('Any valid number was selected');
    end
end

% New
% x = 0; y = 0;
% while (x ~= 1) && (x ~= 2)
%     x = input('Select 1 for ExitationM and 2 for input u3211 : ');
%     if x == 1
%         load ExcitationM
%     elseif x == 2
%         Ts_3ord = 0.001; % Sampling time for u3211
%         Amplitude = 0.1; % Amplitude
%         ExcitationM = u3211_long(Amplitude,Ts_3ord); % Function to generate u3211 input
%         while (y ~= 1) && (y ~= 2)
%             y = input('Select 1 for Task1 and 2 for Task2 : ');
%             if y == 1
%                 ExcitationM = ExcitationM(1:7400,:);
%             end
%         end
%     else
%         disp('Any valid number was selected');
%     end
% end

% Plotting input signal
figure('Name','Input plot')
t_3ord=ExcitationM(:,1);
u_3ord=ExcitationM(:,2);
if x == 1
    u_3ord=u_3ord(t_3ord>26 & t_3ord<100);
    t_3ord=t_3ord(t_3ord>26 & t_3ord<100);
end
plot(t_3ord, u_3ord)
grid
title('Input Signal')
ylabel('u'); ylim([min(u_3ord) - 0.05, max(u_3ord) + 0.05]); xlabel('time [s]')

SetPoint=[0,0];

%% Values selected

t=ExcitationM(:,1);

simulation_time=t(end)-t(1);

%% Launch SIMULATOR

%sim Simulator_Single_Axis
simulation_data = sim('Simulator_Single_Axis','SrcWorkspace', 'current');

%% TASK 1 : Grey-box model identification
% Model ID by grey-box model
% Model call
model_fun = 'drone_model_grey';
% Real Parameter Vector
real_parameters = [Xu; Xq; Mu; Mq; Xd; Md];
% Function call
[identification_grey, error_grey] = Model_identification(simulation_data,sample_time,model_fun,real_parameters);

%% TASK 2 : Black-box model identification
% Input Assigned
t_3ord=0:sample_time:simulation_time;
y_3ord = simulation_data.q;
u_3ord=simulation_data.Mtot;
if x == 1 
    u_3ord=u_3ord(t_3ord>26 & t_3ord<100);
    t_3ord=t_3ord(t_3ord>26 & t_3ord<100);
    y_3ord = simulation_data.q(t_3ord>26 & t_3ord<100);
end
Ts_3ord = sample_time;
% Spectral Figure and determination of the cutting frequency
spectral_analysis(y_3ord,Ts_3ord);
%%
fc = input('Select the cutting frequency of the output : ');
[u_3ord,y_3ord] = bettering_solution(u_3ord,y_3ord,fc,Ts_3ord);
%% Definition of the better p value for system from output error
n = 3; % Order of the system
min_p = 2*n; max_p = 40; 
p = better_p(u_3ord,y_3ord,min_p,max_p,3,Ts_3ord,t_3ord);
disp('The minimum error on the output is on p :'); disp(p);
%% PBSID 
[D_PBSID,S_PBSID,V_PBSID,Y,N] = pbsid_1part(u_3ord,y_3ord(:,1),p);
n = input("looking the graph choose the order detected by the PBSID : ");
[A_PBSID,B_PBSID,C_PBSID,K_PBSID] = pbsid_2part(D_PBSID,n,S_PBSID,V_PBSID,Y,N,p,u_3ord,y_3ord);
% Structuring with greyest
y_id = lsim(ss(A_PBSID,B_PBSID,C_PBSID,D_PBSID), u_3ord, t_3ord);
theta0 = [Xu Xq Mu Mq Xd Md] + 0.01 * (-0.5 + rand(1,6)).*[Xu Xq Mu Mq Xd Md];
model_fun = @drone_model_PBSID;
[identification_PBSID, error_PBSID] = greyest_structuring(u_3ord,y_id,Ts_3ord,theta0,model_fun,real_parameters);
% Graphic comparison of outputs
figure ("Name","Graphic comparison of real outputs wrt identified sys")
plot(t_3ord,y_3ord,'k',t_3ord,lsim(ss(A_PBSID,B_PBSID,C_PBSID,D_PBSID,Ts_3ord), u_3ord, t_3ord),'r');
error_PBSID
error_grey
clear x
%% Delete temporary files

if exist('slprj','dir')
    rmdir('slprj', 's')                                                    
end

%% END OF CODE