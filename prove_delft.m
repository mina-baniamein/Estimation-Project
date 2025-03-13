%% 1 order step input
clc; clear; close all;

% Define state-space matrices
A = 0.8;
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

y = y + 0.1.*rand(1,N); %Noise 

% PBSID-varx
p = 10;
[S,X] = dordvarx(u,y,p,p,'tikh','gcv');
figure, semilogy(S,'*');
%Set system order
order = 1;

x = dmodx(X,order);
[Ai,Bi,Ci,D_delft,Ki] = dx2abcdk(x,u,y,p,p);

real_eig = eig(A) 
identified_eig = eig(Ai) %senza noise i valori coincidono
%% 1 order sinusoidal input
clear all
close all
clc
% Define state-space matrices
A = 0.8;
B = 0.2;
C = 1;
D = 0;
N = 50;  % Number of time steps
T_s = 0.1; % Sampling time
f = 1;

% Define sinusoidal input signal
n = 0:N-1;
u = sin(2 * pi * f * T_s * n);

% Initialize state and output
x = zeros(1, N);
y = zeros(1, N);

% Compute the system response using state-space equations
for k = 2:N
    x(k) = A * x(k-1) + B * u(k); % State update
    y(k) = C * x(k) + D * u(k);   % Output equation
end

y = y + 0.1.*rand(1,N); %noise (senza noise addirittura può peggiorare... Dipende da valori rand)

% PBSID-varx
p = 10;
[S,X] = dordvarx(u,y,p,p,'tikh','gcv');
figure, semilogy(S,'*');
%Set system order
order = 1;

x = dmodx(X,order);
[Ai,Bi,Ci,D_delft,Ki] = dx2abcdk(x,u,y,p,p);

real_eig = eig(A) 
identified_eig = eig(Ai)
%% 3 order simple input
clc; clear; close all;

% Define state-space matrices
A = [0.5 0.1 0;
     0.2 0.6 0.1;
     0   0.3 0.7];

B = [0.2; 0.1; 0.05];
C = [1 0 0];
D = 0;

N = 50;  % Number of time steps

% Define step function input signal
u = ones(1, N);

% Initialize state and output
x = zeros(3, N); % 3 state variables
y = zeros(1, N);

% Compute the system response using state-space equations
for k = 2:N
    x(:, k) = A * x(:, k-1) + B * u(k); % State update
    y(k) = C * x(:, k) + D * u(k);      % Output equation
end

y = y + 0.1.*rand(1,N); %senza noise grafico singolarità ed eig buoni, con noise TUTTO sballato 
                        % (alzare p non migliora la situazione)

% PBSID-varx
p = 20;
[S,X] = dordvarx(u,y,p,p,'tikh','gcv');
figure, semilogy(S,'*');
%Set system order
order = 3;

x = dmodx(X,order);
[Ai,Bi,Ci,D_delft,Ki] = dx2abcdk(x,u,y,p,p);

eig(A) 
eig(Ai)

%% 3 order sinusoidal
clc; clear; close all;

% Define state-space matrices
A = [0.5 0.1 0;
     0.2 0.6 0.1;
     0   0.3 0.7];

B = [0.2; 0.1; 0.05];
C = [1 0 0];
D = 0;

N = 50;  % Number of time steps
T_s = 0.1; % Sampling time
f = 1; % Frequency of sine wave in Hz

% Define sinusoidal input signal
n = 0:N-1;
u = sin(2 * pi * f * T_s * n);

% Initialize state and output
x = zeros(3, N); % 3 state variables
y = zeros(1, N);

% Compute the system response using state-space equations
for k = 2:N
    x(:, k) = A * x(:, k-1) + B * u(k); % State update
    y(k) = C * x(:, k) + D * u(k);      % Output equation
end

y = y + 0.1.*rand(1,N); %senza noise valori sbagliatissimi, con noise
                          % pure peggio


% PBSID-varx
p = 10;
[S,X] = dordvarx(u,y,p,p,'tikh','gcv');
figure, semilogy(S,'*');
%Set system order
order = 3;


x = dmodx(X,order);
[Ai,Bi,Ci,D_delft,Ki] = dx2abcdk(x,u,y,p,p);

eig(A) 
eig(Ai)