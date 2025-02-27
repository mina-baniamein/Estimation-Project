function [A,B,C,D,K] = pbsid(u,y,p,f)

    % Get the total number of data points
    N = length(y);
    
    % Define the number of parameters in C_Delta_p (assumed size from Z)
    nZ = 2 * (N - p);  % Since z is built from interleaved u and y
    
    % Initialize matrices
    Y = y(p:N-1);  % Target output values
    Phi = zeros(N - p, nZ + 1);  % Regression matrix (including D)
    
    % Construct Phi matrix
    for k = p:N-1
        % Generate Z^(k-1, k-p) using u and y
        Z_kp_k = zetagenCdeltaD(u, y, k, p);  % Get p elements
        
        % Fill the regression matrix Phi
        Phi(k - p + 1, 1:nZ) = Z_kp_k(:)';  % Flatten Z into a row
        Phi(k - p + 1, end) = u(k + p);  % Last column is u(k+p)
    end
    
    % Solve the least squares problem using normal equations
    theta = (Phi' * Phi) \ (Phi' * Y);
    
    % Extract results
    C_Delta_p = reshape(theta(1:nZ), [], 2)';  % Reshape back to C_Delta_p
    D = theta(end);

end



