

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

cfg.odolever = [-0.05; -0.0; 2.0]; %[m]
cfg.installangle = [0; -1.6; 1.2];
cfg.installangle = cfg.installangle * param.D2R;
cfg.cbv = euler2dcm(cfg.installangle);

navstate.vel=[    4.7365
    3.3378
   -0.1094];
navstate.cbn = [    0.8135   -0.5813   -0.0142
    0.5815    0.8135    0.0115
    0.0049   -0.0176    0.9998];
wnb_b =[ 0.0339
    0.0035
    0.4054];

vel_pre = cfg.cbv * (navstate.cbn' * navstate.vel + skew(wnb_b) * cfg.odolever)


