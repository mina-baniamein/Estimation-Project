%% 1 order step input
clc; clear; close all;

% Define state-space matrices
A = -0.8;
B = 0.2;
C = 1;
D = 0;
N = 50;  % Number of time steps

% Define input signal (Step function)
u = ones(1, N);

% Initialize state and output
x = zeros(1, N);
y = zeros(1, N);

% Compute the system response using state-space equations
for n = 2:N
    x(n) = A * x(n-1) + B * u(n); % State update
    y(n) = C * x(n) + D * u(n);   % Output equation
end

y = y + 0.01.*rand(1,N); %Noise 

% PBSID-varx
p = 10;
[S,X] = dordvarx(u,y,p,p,'tikh','gcv');
figure, semilogy(S,'*');
%Set system order
order = 1;

x = dmodx(X,order);
[Ai,Bi,Ci,Di,Ki] = dx2abcdk(x,u,y,p,p);


real_eig = eig(A); 
identified_eig = eig(Ai); %senza noise i valori coincidono
%% 
clc
t = Ts:Ts:Ts*N;

Ts = 0.02;
model_fun = 'model1';
[identification]= structuring(Ai,Bi,Ci,Di,model_fun,u,t,Ts)



function [A,B,C,D] = model1(params,Ts)

A = params(1);
B = params(2);
C = params(3);
D = params(4);

end

function [identification]= structuring(Ai,Bi,Ci,Di,model_fun,u,t,Ts)
%continous system conversion
sysd = ss(Ai,Bi,Ci,Di,Ts);
sysc = d2c(sysd);
%output simulation
y = lsim(sysc,u,t);

y = y';

% Data ordering and bring those in frequency domain
sim_data = iddata(y, u, Ts);
data_fd = fft(sim_data);% output of the simulation in the frequency domain

% data_fd = spa(sim_data);

% Initial guess for the identification
guess = [-1,0.1,0.8,0];
sys_init = idgrey(model_fun, guess, 'c');

% Actual Model Identification
identification = struct;
estimated_model = greyest(data_fd, sys_init);
identification.parameters = estimated_model.Report.Parameters.ParVector;
identification.fit = estimated_model.Report.Fit.FitPercent;
identification.fpe = estimated_model.Report.Fit.FPE;
identification.covariance = getcov(estimated_model);
identification.matrix={estimated_model.A; estimated_model.B; estimated_model.C; estimated_model.D};
identification.estimated_model=estimated_model;
end

