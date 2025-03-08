%% 
% Dead Recoding simulation.
% Please run 'test_SINS_trj.m' to generate 'trj10ms.mat' beforehand!!!
% See also  test_SINS_trj, test_SINS_DR.
% Copyright(c) 2009-2014, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 24/02/2012, 08/04/2014
glvs
trj = trjfile('trj10ms.mat'); %获得avp, imu, avp0, wat, ts, repeats
[nn, ts, nts] = nnts(2, trj.ts);
%inst = [0;0;0.5]*glv.min;  kod = 1;  qe = 0; dT = 0;  % od parameters
inst = [0;0;0.5]*glv.deg;  kod = 1.5;  qe = 0; dT = 0;  % od parameters
%inst = [0;0;0.5];  kod = 1;  qe = 0; dT = 0;  % od parameters
%5*pi/180/60=0.0015
%安装偏差角度设置  inst[俯仰，横滚，航向]安装偏差角度
% 刻度系数误差为0，量化误差为0，里程计与IMU时间延迟为0
trjod = odsimu(trj, inst, kod, qe, dT, 0);
%使用 odsimu函数 输入真实的imu和avp信息、里程仪的参数，产生里程仪的测量数据
imuerr = imuerrset(0.01, 50, 0.001, 5);
%imuerr = imuerrset(0.001, 1);  %陀螺三轴零偏，加速计三轴零偏
imu = imuadderr(trjod.imu, imuerr);

davp = avperrset([0;0;0], 0, 0);  %姿态attitude 速度velocity，位置position误差设置
%davp = avperrset([0;0;0], 0, 0);
%dinst = [15;0;10]*glv.min; dkod = 0.2; 
%dinst = [0;0;0]*glv.min; dkod = 0.25; 
dinst = [0;0;0.5]*glv.deg; dkod = 0.2; 
%航向安装偏差角度
%dr = drinit(avpadderr(trjod.avp0,davp), inst+dinst, kod*(1+dkod), ts); % DR init
dr = drinit(avpadderr(trjod.avp0,davp), inst+dinst, kod*(1+dkod), ts);
len = length(imu); avp = prealloc(fix(len/nn), 10);
ki = timebar(nn, len, 'DR simulation.');



for k=1:nn:len-nn+1
    k1 = k+nn-1;
    %wm = imu(k:k1,1:3);  dS= sum(trjod.od(k:k1,1)); t = imu(k1,end);
    wm = imu(k:k1,1:3);  dS= sum(trjod.od(k:k1,1)); t = imu(k1,end);
    %dr = drupdate(dr, wm, dS); 
    dr = drupdate(dr, wm, dS); 
    avp(ki,:) = [dr.avp; t]';
    ki = timebar;
end
dr.distance;
%sum(trjod.od(:,1));

% avperr = avpcmp(avp, trjod.avp);  %DR解算和仿真轨迹的误差对比
% insplot(avp, 'DR', trj.avp); %dr仿真出来的avp和轨迹数据
% inserrplot(avperr);
% 
% simuerr = avpcmp(trj.avp, trjod.avp);   %真实轨迹与仿真轨迹的偏差
% inserrplot(simuerr);



delta = (dinst*Time).* seg().wat(2);


%% mycode
%% 计算 DR 和真实轨迹的最终经纬度
position_finalDR = avp(end, end-3:end-1);  % DR trajectory
position_finalDR(1:2) = position_finalDR(1:2) * (180 / pi);  % 转换为度

position_finalTrue = trjod.avp(end, end-3:end-1);  % True trajectory
position_finalTrue(1:2) = position_finalTrue(1:2) * (180 / pi);  % 转换为度

% 参考点 O（可以是初始位置或指定点）
positionO = trjod.avp(1, end-3:end-1);  % 初始位置
positionO(1:2) = positionO(1:2) * (180 / pi);

%% 调用封装的函数计算夹角
%angle_between_trajectories = calculate_angle(positionO(1:2), position_finalDR(1:2), position_finalTrue(1:2));                                              
                                           
% 显示结果
%fprintf('DR 和真实轨迹之间的夹角为：%.6f 度\n', angle_between_trajectories);


distanceAB = haversine_distance(position_finalDR(1:2), position_finalTrue(1:2));
distanceOA = haversine_distance(position_finalDR(1:2),positionO(1:2));
distanceOB = haversine_distance(position_finalTrue(1:2),positionO(1:2));
delta_OAB = distanceOA/distanceOB - 1;
fprintf('计算得到刻度系数：%.10f ', delta_OAB);
% 计算夹角
cos_angle = (distanceOA^2 + distanceOB^2 - distanceAB^2) / (2 * distanceOA * distanceOB);
angle = acos(cos_angle);  % 夹角 (弧度)

% 转换为角度并输出结果
angle_deg = rad2deg(angle);
fprintf('夹角：%.6f 度\n', angle_deg);
close all
%% 调用封装的函数计算航向安装偏差角
[delta_OAB, angle_deg] = calculate_heading_bias(positionO(1:2), position_finalDR(1:2), position_finalTrue(1:2));
% 输出结果
fprintf('计算得到刻度系数：%.10f\n', delta_OAB);
fprintf('计算得到航向安装偏差角：%.6f 度\n', angle_deg);