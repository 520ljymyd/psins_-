%仿真产生imu数据，avp数据，真实无噪声
% Trajectory generation for later simulation use.
% See also  test_SINS, test_SINS_GPS_153, test_DR.
% Copyright(c) 2009-2014, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 10/06/2011, 10/02/2014
glvs
ts = 0.01;       % sampling interval 100Hz采样率
%Time = 27.8*3600;
Time = 900;
avp0 = [[0;0;0];  [0;0;0]; [45.7796*glv.deg; 126.6705*glv.deg; 0]]; % init avp
% trajectory segment setting
xxx = [];
% seg = trjsegment(xxx, 'init',         0);   %初始化
% seg = trjsegment(seg, 'uniform',      900);
seg = trjsegment(xxx, 'init',       0);   %初始化
% % seg = trjsegment(seg, 'uniform',      Time);
seg = trjsegment(seg, 'accelerate',   3, xxx, 1000);
seg = trjsegment(seg, 'headdown', 10, 2);
seg = trjsegment(seg, 'descent',      5, 0.5, xxx, 10);
seg = trjsegment(seg, 'climb',        5, 2.5, xxx, 20);
% %seg = trjsegment(seg, 'uniform',      Time-1);     %注释此行，进入gps153
% seg = trjsegment(seg, 'uniform',      Time-100);%
% seg = trjsegment(seg, 'coturnleft',   45, 2, xxx, 4);%
% seg = trjsegment(seg, 'uniform',      100);         %注释此行进入DR标定
%% ignore
% seg = trjsegment(seg, 'coturnright',  10*5, 9, xxx, 4);
% seg = trjsegment(seg, 'uniform',      100);
%seg = trjsegment(seg, 'climb',        50, 2, xxx, 50);
seg = trjsegment(seg, 'uniform',      10);

% seg = trjsegment(seg, 'descent',      50, 2, xxx, 50);
% seg = trjsegment(seg, 'uniform',      100);
% seg = trjsegment(seg, 'deaccelerate', 5,  xxx, 2);
% seg = trjsegment(seg, 'uniform',      100);
%generate, save & plot
trj = trjsimu(avp0, seg.wat, ts, 1);
trjfile('trj10ms.mat', trj);

insplot(trj.avp);   %绘图  avp trajectory
imuplot(trj.imu);   %绘图   imu data
pos2gpx('trj_SINS_gps', trj.avp(1:round(1/trj.ts):end,7:9)); % to Google Earth
%seg.wat :包含持续时间、机头前向速度、姿态变化率以及"t frame”中的加速度
