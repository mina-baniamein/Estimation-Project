function [A,B,K] = pbsid2(D,n,S_svd,V_svd,Phi,Y,N,p,u,y)

%Second section of the pbsid algorithm for a SISO system
%Requires Z_k2_k1 to be run
%Inputs: estmated D, n order, SV matrixes from decomposition, Phi matrix
%        assembled in previous section
%Outputs: estmated A,B,C and K matrixes of the system

% State estimation
X_est_n = sqrtm(S_svd(1:n, 1:n)) * V_svd(:, 1:n)';
u_C = Phi(:,end);
 
% C estimation
b_C = Y - D.*u_C;
A_C = X_est_n';
 
C = lsqr(A_C,b_C);

% A,B and K estimation
e = zeros(N-2-p,1);
b_ABK = zeros(2*(N-2-p),1);
index = 1;
A_ABK = zeros(2*(N-2-p),8);
 
for i = p : N-2
    e(index) = y(i) - C' * X_est_n(:,i-p+1) - D* u(i);
     
    b_ABK(2*index-1,1) = X_est_n(1,index+1);
    b_ABK(2*index,1) = X_est_n(2,index+1);

    A_ABK(2*index-1,:) = [[X_est_n(:,index)', 0,0] , [u(i),0], [e(index),0]];
    A_ABK(2*index,:)   = [[X_est_n(:,index)', 0,0] , [u(i),0], [e(index),0]];
     
    index = index + 1;
 end

ABK = lsqr(A_ABK, b_ABK);

A = reshape(ABK(1:4),2,2);
B = ABK(5:6);
K = ABK(7:8);

end
