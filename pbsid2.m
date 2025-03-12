function [A,B,K] = pbsid2(D,n,S_svd,V_svd,Phi,Y,N,p,u,y)

%Second section of the pbsid algorithm for a SISO system
%Requires Z_k2_k1 to be run
%Inputs: estmated D, n order, SV matrixes from decomposition, Phi matrix
%        assembled in previous section
%Outputs: estmated A,B,C and K matrixes of the system


% State estimation
X_est_n = sqrtm(S_svd(1:n, 1:n)) * V_svd(:, 1:n)';
u_C = u(p:N-1) ;%Phi(:,end);

% C estimation
b_C = Y - D*u_C;
A_C = X_est_n';
C = (A_C'*A_C)\(A_C'*b_C);
%C = lsqr(A_C,b_C);

% A,B and K estimation
e = zeros(N-2-p,1);
b_ABK = zeros(2*(N-2-p),1);
index = 1;
A_ABK = zeros(2*(N-2-p),n^2+2*n);
A_a_mini = zeros(n,n^2);

for i = p : N-2
    e(index) = y(i) - C' * X_est_n(:,i-p+1) - D* u(i);

    b_ABK (n*index-n+1 : n*index,1) = X_est_n(1:n,index+1);

    for j = 1 : n
        A_a_mini(j,j*n-n+1 : j*n) = X_est_n(:,index)';
        A_b_mini = eye(n)*u(i);
        A_k_mini = eye(n)*e(index);
    end

    A_ABK(n*index-n+1 : n*index, :) = [A_a_mini,A_b_mini,A_k_mini];

    index = index + 1;
 end

ABK = lsqr(A_ABK, b_ABK);

A = reshape(ABK(1:n^2),n,n);
B = ABK(n^2+1:n^2+n);
K = ABK(n^2+n+1:end);

end
