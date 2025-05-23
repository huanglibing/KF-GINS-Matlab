

% cfg.installangle = [0; -1.6; 1.2];
% cfg.installangle = cfg.installangle * param.D2R;
% cfg.cbv = euler2dcm(cfg.installangle);
% 
% navstate.vel=[    0.1613
%   -14.5356
%    -0.0564];
% navstate.cbn = [    0.0321    0.9994   -0.0163
%    -0.9992    0.0325    0.0241
%     0.0246    0.0155    0.9996];
% 
% vel_pre = cfg.cbv * (navstate.cbn' * navstate.vel )

cfg.odolever = [0; 0; 0]; %[m]
cfg.installangle = [0; -0.1; -0.75]; %[deg]
cfg.installangle = cfg.installangle * param.D2R;
cfg.cbv = euler2dcm(cfg.installangle);

navstate.vel=[    0.0195
   -0.8751
    0.0073];
navstate.cbn = [    0.0093    1.0000   -0.0026
   -0.9999    0.0093    0.0094
    0.0094    0.0025    1.0000];
wnb_b =[   -0.0141
    0.0345
    0.0118];

vel_pre = cfg.cbv * (navstate.cbn' * navstate.vel + skew(wnb_b) * cfg.odolever)


