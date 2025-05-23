% 数据一 安装角寻找 杆臂寻找
% 找到一个点 直线运动 通过不断调整安装矩阵来使得vel_pre逼近[x 0 0]
% 再找到一个点 转弯动作 通过不断调整杆臂来使得vel_pre逼近[x 0 0]

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


