% 数据一 安装角寻找 杆臂寻找
% 找到一个点 直线运动 通过不断调整安装矩阵来使得vel_pre逼近[x 0 0]
% 再找到一个点 转弯动作 通过不断调整杆臂来使得vel_pre逼近[x 0 0]

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

cfg.odolever = [-0.05; -0.0; 2.0]; %[m] 前右下 以imu为原点的坐标系
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


