function [] = spectral_analysis(y,Ts)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Estimation and Learning in Aerospace Project A.Y. 24-25 
% Function to do a spectral analysis for the choice of the cutting
% frequency

% Authors:  Alessandro Castelanelli Oddo (alessandro.castelanelli@polimi.it)
%           Mina Baniamein (mina.baniamein@polimi.it)                     
%            (@polimi.it)                                                   
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
fs = 1/Ts; % Sampling frequency
n = length(y); 
frequenze = linspace(0, fs/2, floor(n/2));

% Fourier's Transform
Y_fft = abs(fft(y)); 
Y_fft = Y_fft(1:floor(n/2)); % Positive half

% Spectral analysis plot
figure;
plot(frequenze, Y_fft, 'b', 'LineWidth', 1.5);
xlabel('Frequency (Hz)');
ylabel('Magnitude');
title('Spectral analysis of the signal');

end