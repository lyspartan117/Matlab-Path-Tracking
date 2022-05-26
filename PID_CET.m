function [x,y,phi,delta_f,i] = PID_CET(k,init,target,noise,eps,maxn)
%PID_CET 根据车辆横向追踪误差的PID，基于后轮为中心
%   控制量为a和delta_f,其中a为固定值0
%   k:kp,ki,kd比例，积分，微分系数所构成的向量,[kp,ki,kd]
%   init:初始位置信息，分别为初始时x坐标，y坐标，航向角phi，车辆长度l的向量，v初始速度信息,[x,y,phi,l,v];
%   target:目标信息，[dt,tt_x,tt_y],包含了采样时间dt，目标曲线的信息对应的(tt_x,tt_y)，目标曲线应为插值后的结果
%   eps,判断离最终目标点距离为多少时停止追踪，默认1e-1
%   maxn,最大跟踪点的数量，默认和目标坐标的向量长度相同
%   返回车辆运行状态的位置信息,x,y,phi,delta_f,i,i对应跟踪结束的x,y中的坐标
%   干扰信息，暂时空缺

% 读入输入
% 目标tt_x,tt_y向量长度
n = size(target{2},2); % x,y最多会有n的数据点
if nargin <= 4
    eps = 1e-1;
    maxn = n;
elseif nargin <= 5
    maxn = n;
end
% kp,ki,kd
kp = k(1); 
ki = k(2);
kd = k(3);
% 初始x,y,phi,l,v位置信息
x = linspace(0,0,n);
x(1) = init(1);
y = linspace(0,0,n);
y(1) = init(2);
phi = linspace(0,0,n);
phi(1) = init(3);
l = init(4);
v = init(5);
% 转向角初始化
delta_f = linspace(0,0,n); % 前轮转向角，初始值默认为0
delta_f(1) = 0;
% 加速度默认为0
a = 0;
% 读取采样长度，跟踪曲线tt_x,tt_y信息
dt = target{1};
tt_x = target{2};
tt_y = target{3};
% noise暂不管

err_now = 0; % 当前误差
err_sum = 0; % 累计误差
for i = 1:1:maxn
    % 根据当前位置寻找目标曲线最接近的点
    distance = (tt_x - x(i)).^2 + (tt_y - y(i)).^2;
    % 和轨迹最终点距离很小，结束程序
    if (distance(n) < eps)
        return;
    end
    [~,point] = min(distance);
    
    % 计算目前位置的误差值
    err_pre = err_now;
    if (point + 1 > n)
        err_now = calcERR_CET([x(i),y(i)],[tt_x(point),tt_y(point)],phi(i),[2 * tt_x(point) - tt_x(point - 1),2 * tt_y(point) - tt_y(point - 1)]);
    else
        err_now = calcERR_CET([x(i),y(i)],[tt_x(point),tt_y(point)],phi(i),[tt_x(point + 1),tt_y(point + 1)]);
    end
    err_sum = err_sum + err_now;
    % 根据当前误差值更新控制量delta_f
    delta_f(i + 1) = kp * err_now + ki * err_sum + kd * (err_now - err_pre);
    % 限制delta_f的大小，防止出现过大的情况
    if delta_f(i + 1) > pi / 6
        delta_f(i + 1) = pi / 6;
    elseif delta_f(i + 1) < -pi / 6
        delta_f(i + 1) = - pi / 6;
    end
    
    % 计算更新delta_f后的位置信息
    x(i + 1) = x(i) + v * cos(phi(i)) * dt; % x
    y(i + 1) = y(i) + v * sin(phi(i)) * dt; % y
    phi(i + 1) = phi(i) + v * delta_f(i + 1) / l * dt; % 航向角
    v = v + a * dt;
end
end

