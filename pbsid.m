function [A,B,C,D,K] = pbsid(u,y,p,f)

    % Get the total number of data points
    N = length(y);
    
    % Define the number of parameters in C_Delta_p (assumed size from Z)
    % nZ = 2 * (N - p);  % Since z is built from interleaved u and y
    
    % Initialize matrices
    Y = y(p:N-1);  % Target output values
    Phi = zeros(N - p, 2*p+1);  % Regression matrix (including D)
    
    % Construct Phi matrix
    for k = (p+1):N
        % Generate Z^(k-1, k-p) using u and y
        Z_kp_k = zetagenCdeltaD(u, y, k, p);  % Get p elements
        nZ_kp_k = length(Z_kp_k);

        % Fill the regression matrix Phi
        Phi(k - p, 1:nZ_kp_k) = Z_kp_k(:)';  % Flatten Z into a row
        Phi(k - p, end) = u(k);  % Last column is u(k)
    end
    
    % Solve the least squares problem using normal equations
    theta = (Phi' * Phi) \ (Phi' * Y);
    
    % % Extract results
    C_Delta_p = theta(1:(length(theta)-1);  % Reshape back to C_Delta_p
    D = theta(end);

end


