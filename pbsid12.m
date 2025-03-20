function [D,S_svd,V_svd,Y,N] = pbsid12(u,y,p)
%%ANTI-DIAG GAMMADELTA GIUSTO
%First section of the pbsid algorithm for a SISO system
%Requires Z_k2_k1 to be run
%Inputs: command u and output measuraments y of the system
%Outputs: D matrix and singular values plot for n assumption

% Get the total number of data points
N = length(y);

% Define the number of parameters in C_Delta_p (assumed size from Z)
% nZ = 2 * (N - p);  % Since z is built from interleaved u and y

% Initialize matrices
Y = y(p:N-1);  % Target output values

Phi = zeros(N - p - 1, 2*p+1);  % Regression matrix (including D)

%Z generation  
for k = p:N-1

    % Generate Z^(k-1, k-p) using u and y
    Z_kp_k = Z_k2_k1(u, y, k-1, k-p);  % Get p elements

    % Fill the regression matrix Phi
    Phi(k - p+1, 1:2*p) = Z_kp_k(:)';  % Flatten Z into a row
    Phi(k - p+1, end) = u(k);  % Last column is u(k)

end

x = lsqr(Phi,Y);

% Extract results
C_Delta_p = x(1:(length(x)-1));  % Reshape back to C_Delta_p
D = x(end);
% f = p assumption
Gamma_Delta_pp = zeros(p,2*p);
C_Delta_p = [C_Delta_p', zeros(1,2*p)];

for i = 1:p
    Gamma_Delta_pp(i,:) = C_Delta_p(2*i-1: 2*i + 2*p -2);
end


Z = [];

for k = 0 : N-p-1
    Z_temp = Z_k2_k1(u, y, p+k-1, k);
    Z = [Z, Z_temp];
end

[~, S_svd, V_svd] = svd(Gamma_Delta_pp * Z , 'econ');

figure
plot(diag(S_svd), 'o')
end
