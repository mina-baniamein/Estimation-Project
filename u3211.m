clear all, close all, clc
% %function [u]
% 
% ts = 0.0040;


% Define time values
% t = [0 1    3    5    6    7 8]; 
t = [0:0.0040:8]';
% u = [0 0.1 -0.1  0.1 -0.1  0 0];
sec1 = ones((1/0.004),1);
sec3 = ones((3/0.004),1);
sec2 = ones((2/0.004),1);
sec05 = ones((0.5/0.004),1);

u0 = 0.*sec05;
u1 = 0.1*sec3;
u2 = -0.1*sec2;
u3 = 0.1*sec1;
u4 = -u3;
u5 = u0;

u = [u0;u1;u2;u3;u4;u5;0];


% Plot using stairs
figure;
plot(t, u, 'b');
xlabel('Time [s]');
ylabel('u');
grid on;

u3211 = [t,u]