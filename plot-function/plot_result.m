% -------------------------------------------------------------------------
% KF-GINS-Matlab: An EKF-based GNSS/INS Integrated Navigation System in Matlab
%
% Copyright (C) 2024, i2Nav Group, Wuhan University
%
%  Author : Liqiang Wang
% Contact : wlq@whu.edu.cn
%    Date : 2022.11.30
% -------------------------------------------------------------------------
close all
% importdata navresult

if cfg.outputfolder=="dataset1"
    navpath = "dataset1/NavResult.nav";
    navdata = importdata(navpath);
    truthpath = 'dataset1/truth.nav';
    truthdata=importdata(truthpath);
elseif cfg.outputfolder=="dataset2"
    navpath = "dataset2/NavResult_GNSSVEL.nav";
    navdata = importdata(navpath);
    truthpath = 'dataset2/truth.nav';
    truthdata=importdata(truthpath);
else
    navpath = "dataset3/NavResult_ODONHC.nav";
    navdata = importdata(navpath);
    truthpath = 'dataset3/truth.nav';
    truthdata=importdata(truthpath);
end

% check heading error, 航向角误差处理
for i = 1:size(truthdata(:, 11), 1)
    if truthdata(i, 11) > 180
        truthdata(i, 11) = truthdata(i, 11) - 360;
    end
    if truthdata(i, 11) < -180
        truthdata(i, 11) = truthdata(i, 11) + 360;
    end
end

% velocity
figure()
plot(navdata(:, 2), navdata(:, 6:8));
title('Velocity');
legend('North', 'East', 'Down');
xlabel('Time[s]');
ylabel('Vel[m/s]');
grid("on");

% attitude
figure()
subplot(311);hold on;grid on;plot(navdata(:, 2), navdata(:, 9));plot(truthdata(:, 2), truthdata(:, 9),'--');legend('Roll','ref');
subplot(312);hold on;grid on;plot(navdata(:, 2), navdata(:,10));plot(truthdata(:, 2), truthdata(:, 10),'--');legend('Pitch','ref');
subplot(313);hold on;grid on;plot(navdata(:, 2), navdata(:, 11));plot(truthdata(:, 2), truthdata(:, 11),'--');legend( 'Yaw','ref');
title('Attitude');
xlabel('Time[s]');
ylabel('Att[deg]');
grid("on");

% position
param = Param();
blh = navdata(:, 3:5);
blh(:, 1) = blh(:, 1) * param.D2R;
blh(:, 2) = blh(:, 2) * param.D2R;
first_blh = blh(1, 1:3);

[rm, rn] = getRmRn(first_blh(1), param);
h = first_blh(2);
DR = diag([rm + h, (rn + h)*cos(first_blh(1)), -1]);

% blh to ned
pos = zeros(size(blh));
for i = 1:size(pos, 1)
    delta_blh = blh(i, :) - first_blh;
    delta_pos = DR * delta_blh';
    pos(i, :) = delta_pos';
end


blhRef = truthdata(:, 3:5);
blhRef(:, 1) = blhRef(:, 1) * param.D2R;
blhRef(:, 2) = blhRef(:, 2) * param.D2R;
first_blh = blhRef(1, 1:3);

[rm, rn] = getRmRn(first_blh(1), param);
h = first_blh(2);
DR = diag([rm + h, (rn + h)*cos(first_blh(1)), -1]);

% blhRef to ned
truthpos = zeros(size(blhRef));
for i = 1:size(truthpos, 1)
    delta_blh = blhRef(i, :) - first_blh;
    delta_pos = DR * delta_blh';
    truthpos(i, :) = delta_pos';
end

%% plane position
figure()
hold on;
plot(pos(:, 2), pos(:, 1));
plot(truthpos(:, 2), truthpos(:, 1),'--');
legend( 'Pos','ref');
title('Position','ref');
xlabel('East[m]');
ylabel('North[m]');
grid("on");

%% height
figure()
hold on;
plot(navdata(:, 2), navdata(:, 5));
plot(truthdata(:, 2), truthdata(:, 5),'--');
title('Height','ref');
xlabel('Time[s]');
ylabel('Height[m]');
grid("on");





