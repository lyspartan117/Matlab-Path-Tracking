clear
clc

%% 直线
% kp,ki,kd系数输入
kp = 2;
ki = 0;
kd = 20;
k = [kp,ki,kd];
% 初始x,y和phi,l,v,dt信息
x = 0;
y = 0;
phi = pi / 3;
l = 2;
v = 2;
init = [x,y,phi,l,v];% 初始值
dt = 0.1; % 最大时间间隔内距离v * dt
% 目标一条直线取点
tt_x = linspace(0,100,1000);
tt_y = linspace(10,10,1000);
target = {dt,tt_x,tt_y};
% 画出目标曲线
plot(tt_x,tt_y,'.',tt_x,tt_y,'b-');
hold on;
% PID控制
[r_x,r_y,r_phi,r_delta_f,i] = PID_CET(k,init,target,0);
% 画出PID控制结果
plot(r_x(1:i),r_y(1:i),'.',r_x(1:i),r_y(1:i),'r-');
% 画出结果
% DrawResult(r_x(1:i),r_y(1:i),r_phi(1:i),r_delta_f(1:i),l,tt_x,tt_y);
%% 正弦线
% kp,ki,kd系数输入
kp = 2;
ki = 0.25;
kd = 20;
k = [kp,ki,kd];
% 初始x,y和phi,l,v,dt信息
x = 0;
y = 0;
phi = 0;
l = 2;
v = 2;
init = [x,y,phi,l,v];% 初始值
dt = 0.1; % 最大时间间隔内距离v * dt
% 目标一条直线取点
tt_x = linspace(0,100,1000);
tt_y = sin(tt_x / 3.0);
target = {dt,tt_x,tt_y};
% 画出目标曲线
plot(tt_x,tt_y,'.',tt_x,tt_y,'b-');
hold on;
% PID控制
[r_x,r_y,r_phi,r_delta_f,i] = PID_CET(k,init,target,0);
% 画出PID控制结果
plot(r_x(1:i),r_y(1:i),'.',r_x(1:i),r_y(1:i),'r-');
% DrawResult(r_x(1:i),r_y(1:i),r_phi(1:i),r_delta_f(1:i),l,tt_x,tt_y);
%% 圆曲线
% kp,ki,kd系数输入
kp = 2;
ki = 0.02;
kd = 20;
k = [kp,ki,kd];
% 初始x,y和phi,l,v,dt信息
x = 6;
y = 0.1;
phi = pi / 2;
l = 2;
v = 2;
init = [x,y,phi,l,v];% 初始值
dt = 0.1; % 最大时间间隔内距离v * dt
% 目标一条直线取点
% 圆形采用极坐标
% 半径r
r = 5;
% 得到x
tt_x = r * cos(linspace(0,2*pi,1000)); % 角度从0到2pi,取1000个点
% 得到y
tt_y = r * sin(linspace(0,2*pi,1000));
target = {dt,tt_x,tt_y};
% 画出目标曲线
plot(tt_x,tt_y,'.',tt_x,tt_y,'b-');
hold on;
% PID控制
[r_x,r_y,r_phi,r_delta_f,i] = PID_CET(k,init,target,0);
% 画出PID控制结果
plot(r_x(1:i),r_y(1:i),'.',r_x(1:i),r_y(1:i),'r-');
% DrawResult(r_x(1:i),r_y(1:i),r_phi(1:i),r_delta_f(1:i),l,tt_x,tt_y);