clear
close all
clc
%% Experiment Definition
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
% U3211 input 
u3211_vect = u3211(A,Ts_1ord); % Function to generate u3211 input
u_1ord = u3211_vect(:,2); % u3211 input
t_1ord = u3211_vect(:,1); % Time for u3211 input
% Impulse input
% t_1ord = 0:Ts_1ord:10; % Time for impulse input 
% u_1ord = 0.1*[0;ones(length(t_1ord)-1,1)]; % Impulse input
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
























