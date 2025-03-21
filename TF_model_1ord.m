function G_param = TF_model_1ord(theta)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Estimation and Learning in Aerospace Project A.Y. 24-25 
% Transfer funtion generation of a simple first order example to validate
% PBSID funtioning

% Authors:  Alessandro Castelanelli Oddo (alessandro.castelanelli@polimi.it)
%            (@polimi.it)                     
%            (@polimi.it)                                                   
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% A,B,C,D matricies definition wrt theta parameter
A = [theta(1)]; B = [theta(2)];
C = [theta(3)]; D = [theta(4)];
% State space model
G_param = ss(A, B, C, D); % Continuous time
% Transfer function
G_param = tf(G_param);

end