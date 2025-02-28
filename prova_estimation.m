clc

u = u_value(u_time>(26) & u_time<100);
y1 = simulation_data.q((u_time>26 & u_time<100));
y2 = simulation_data.ax((u_time>26 & u_time<100));

N = length(u);

p = 12;

y = y2;
    % Get the total number of data points
    N = length(y);
    
    % Define the number of parameters in C_Delta_p (assumed size from Z)
    % nZ = 2 * (N - p);  % Since z is built from interleaved u and y
    
    % Initialize matrices
    Y = y(p:N-1);  % Target output values
    Phi = zeros(N - p, 2*p+1);  % Regression matrix (including D)
    
    % % Construct Phi matrix
    % for k = (p+1):N
    %     % Generate Z^(k-1, k-p) using u and y
    %     Z_kp_k = zetagenCdeltaD(u, y, k, p);  % Get p elements
    %     nZ_kp_k = length(Z_kp_k);
    % 
    %     % Fill the regression matrix Phi
    %     Phi(k - p, 1:nZ_kp_k) = Z_kp_k(:)';  % Flatten Z into a row
    %     Phi(k - p, end) = u(k);  % Last column is u(k)
    % end


        % Construct Phi matrix
    for k = (p+1):N
        % Generate Z^(k-1, k-p) using u and y
        Z_kp_k = Z_k2_k1(u, y, k-1, k-p);  % Get p elements
        nZ_kp_k = length(Z_kp_k);

        % Fill the regression matrix Phi
        Phi(k - p, 1:nZ_kp_k) = Z_kp_k(:)';  % Flatten Z into a row
        Phi(k - p, end) = u(k);  % Last column is u(k)
    end
    
    % Solve the least squares problem using normal equations
    % theta = (Phi' * Phi) \ (Phi' * Y);
    
    x = lsqr(Phi,Y);

    % % Extract results
    C_Delta_p = x(1:(length(x)-1));  % Reshape back to C_Delta_p
    D = x(end);

    %%

    f = p; %assumption
    C_Delta_f = C_Delta_p;

    Gamma_f_Delta_p = zeros(length(C_Delta_f), length(C_Delta_p));
    
    % Fill the first row with A
    Gamma_f_Delta_p(1, :) = C_Delta_p';
    
    % Fill the first column with B
    Gamma_f_Delta_p(:, 1) = C_Delta_f;

    for d = 2 : length(C_Delta_p)
        for s = 2 : (d-1)
            q = (d+1 - s);
            Gamma_f_Delta_p(q, s) = C_Delta_p(d);
        end
    end

    % constructing Z

    Z = [];
    for k = 0 : N-p-1
        Z_temp = Z_k2_k1(u, y, p+k, 1+k);
        Z = [Z, Z_temp];
    end

    [~, S_svd, V_svd] = svd(Gamma_f_Delta_p * Z , 'econ');

    figure
    plot(diag(S_svd), 'o')

    %X_est = sqrtm(S_svd(1,:)) * V_svd';

    n = 3; %dal grafico dei singular values (oppure dal modello lineare) sappiamo che n=1

    X_est_n = sqrtm(S_svd(1:n, 1:n)) * V_svd(:, 1:n)';

  

      


    
