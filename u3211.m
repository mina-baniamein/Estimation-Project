function [u3211_vect] = u3211(A,Ts)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Estimation and Learning in Aerospace Project A.Y. 24-25 
% Function creating a sequence u3211 with amplitude as "A" at sample time
% "Ts" in a total time of 8 with an ammount of N samples

% Outputs : u3211 : [N x 2] Sequence u3211 [time,input]

% Authors:  Alessandro Castelanelli Oddo (alessandro.castelanelli@polimi.it)
%            (@polimi.it)                     
%            (@polimi.it)                                                   
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Define time values
t = [0:Ts:8]';
sec1 = ones((1/Ts),1);
sec3 = ones((3/Ts),1);
sec2 = ones((2/Ts),1);
sec05 = ones((0.5/Ts),1);

u0 = 0.*sec05;
u1 = A*sec3;
u2 = -A*sec2;
u3 = A*sec1;
u4 = -u3;
u5 = u0;

u = [u0;u1;u2;u3;u4;u5;0];


% % Plot using stairs
% figure;
% plot(t, u, 'b');
% xlabel('Time [s]');
% ylabel('u');
% grid on;

u3211_vect = [t,u];

end