% %function [identification]= structuring1(A_id,B_id,C_id,D_id,model_fun,u1,t1,Ts)
% %continous system conversion
% sysd = ss(A_id,B_id,C_id,D_id,Ts);
% Gsysd = tf(sysd);
% Gsysc = d2c(Gsysd);
% %output simulation
% y = lsim(Gsysc,u1,t1);
% 
% 
% % Data ordering and bring those in frequency domain
% data_fd = iddata(y, u1, Ts);
% %data_fd = fft(sim_data);% output of the simulation in the frequency domain
% 
% % data_fd = spa(sim_data);
% 
% %% Initial guess for the identification
% guess = [0,0,0,0];
% % sys_init = idgrey(model_fun, guess, 'c');
sysd = ss(A_id,B_id,C_id,D_id,Ts);
sysc = d2c(sysd);
%output simulation
y = lsim(sysc,u1,t1);

% Data ordering and bring those in frequency domain
sim_data = iddata(y, u1, Ts);
data_fd = fft(sim_data);% output of the simulation in the frequency domain

% data_fd = spa(sim_data);

% Initial guess for the identification
guess = [0.8,0.4,0.9,0.1];
sys_init = idgrey(model_fun, guess, 'c');
% Actual Model Identification
identification = struct;
opt = greyestOptions('InitialState','backcast','Display','on','EnforceStability',true);
estimated_model = greyest(data_fd, sys_init);
identification.parameters = estimated_model.Report.Parameters.ParVector;
identification.fit = estimated_model.Report.Fit.FitPercent;
identification.fpe = estimated_model.Report.Fit.FPE;
identification.covariance = getcov(estimated_model);
identification.matrix={estimated_model.A; estimated_model.B; estimated_model.C; estimated_model.D};
identification.estimated_model=estimated_model;
%end