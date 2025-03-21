function err = FDOE_problem(theta, w, G_id,fun)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Estimation and Learning in Aerospace Project A.Y. 24-25 
% Frequency Domain Output Error Problem Definition with the model in input

% Outputs : err : [N:1] errors in magnitude and phase in each step of the
%                       frequencies domain "w"

% Authors:  Alessandro Castelanelli Oddo (alessandro.castelanelli@polimi.it)
%            (@polimi.it)                     
%            (@polimi.it)                                                   
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Phisich model creation
G_param = fun(theta);
% Frequency response evaluation of identified model of PBSID and phisic
% parametrized model
[mag, phase] = bode(G_param, w);
[mag_id,phase_id] = bode(G_id, w);
mag = squeeze(mag); phase = squeeze(phase);
mag_id = squeeze(mag_id); phase_id = squeeze(phase_id);
% Logaritmic Normalization (to reduce errors)
err_mag = (20*log10(mag) - 20*log10(mag_id)) / max(abs(20*log10(mag_id)));
err_phase = (phase - phase_id) / max(abs(phase_id));
% Arrays of errors
err = [err_mag; err_phase];

end