% -------------------------------------------------------------------------
% KF-GINS-Matlab: An EKF-based GNSS/INS Integrated Navigation System in Matlab
%
% Copyright (C) 2024, i2Nav Group, Wuhan University
%
%  Author : Liqiang Wang
% Contact : wlq@whu.edu.cn
%    Date : 2023.3.9
% -------------------------------------------------------------------------

function kf = ODONHCUpdate(navstate, odonhc_vel, kf, cfg, thisimu, dt)

    param = Param();

    %% measurement innovation 计算车体角速度（w_nb_b）
    wib_b = thisimu(2:4, 1) / dt;
    wie_n = [param.WGS84_WIE * cos(navstate.pos(1)); 0; -param.WGS84_WIE * sin(navstate.pos(1))];
    wen_n = [navstate.vel(2) / (navstate.Rn + navstate.pos(3)); 
            -navstate.vel(1) / (navstate.Rm + navstate.pos(3)); 
            -navstate.vel(2) * tan(navstate.pos(1)) / (navstate.Rn + navstate.pos(3))];
    win_n = wie_n + wen_n;
    wnb_b = wib_b - navstate.cbn' * win_n;

    vel_pre = cfg.cbv * (navstate.cbn' * navstate.vel + skew(wnb_b) * cfg.odolever);
    Z = vel_pre - odonhc_vel;

    %% measurement equation and noise
    R = diag(power(cfg.odonhc_measnoise, 2));%m m m
    H = zeros(3, kf.RANK);

    % 唐海亮论文参考 十五阶
    % H(1:3, 4:6) = navstate.cbn'; % 此处论文少写了一个旋转矩阵 cfg.cbv*
    % H(1:3, 7:9) = -cfg.cbv * navstate.cbn' * skew(navstate.vel);
    % H(1:3, 10:12) = -cfg.cbv * skew(cfg.odolever);     

    % 吴佳豪论文参考 二十一阶
    H(1:3, 4:6) = cfg.cbv*navstate.cbn';
    H(1:3, 7:9) = -cfg.cbv * navstate.cbn' * skew(navstate.vel);
    H(1:3, 10:12) = -cfg.cbv * skew(cfg.odolever);      
    %H(1:3, 16:18) = -cfg.cbv * skew(cfg.odolever) * diag(wib_b);  %为什么增加这一项更差了

    if cfg.usenhc
        Z = Z(2:3);
        R = R(2:3,2:3);
        H = H(2:3,:);
    end

    %% update
    K = kf.P * H' / (H * kf.P * H' + R);
    kf.x = kf.x + K*(Z - H*kf.x);
    kf.P=(eye(kf.RANK) - K*H) * kf.P * (eye(kf.RANK) - K*H)' + K * R * K';

end