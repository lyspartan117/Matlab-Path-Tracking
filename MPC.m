function [x,y,phi,v,delta_f,ERR] = MPC(init,target,MaxErr,Urange)
%MPC 模型预测控制
%   init:初始位置信息，分别为初始时x坐标，y坐标，航向角phi，车辆长度l的向量，v初始速度信息,[x,y,phi,l,v];
%   输入角度均需要满足[0,2 * pi)
%   target:目标信息，[dt,tt_x,tt_y,tt_v],包含了采样时间dt，目标曲线的信息对应的(tt_x,tt_y)，目标速度tt_v
%   返回车辆运行状态的位置信息,x,y,phi,delta_f,i,i对应跟踪结束的x,y中的坐标
%   maxErr最大误差,防止陷入死循环,-1则代表未输入,则默认为初始点到所有点中距离最大值
%   Urange为控制量约束，为可选参数，输入按照{umin,umax,delta_u_min,delta_u_max}输入，其中均为列向量
%   返回跟踪过程中x,y,phi,v,delta_f信息,ERR为当误差大于MaxErr时退出时的信息，-1代表误差过大导致退出循环
%% 输入参数处理
n = size(target{2},2); % 共有n的数据点
% 实际位置[act_x,act_y,act_phi]
act_pos = zeros(3,n);
act_pos(1,1) = init(1);
act_pos(2,1) = init(2);
act_pos(3,1) = init(3);
% 车辆长度
l = init(4);
% 初始速度
init_v = init(5);
% 车辆实际控制量[v,delta_f]
act_ctl = zeros(2,n);
act_ctl(1,1) = 0; % 初始v控制量
act_ctl(2,1) = 0; % 初始delta_f控制量
% 读取采样时间长度
dt = target{1};

%% 生成参考轨迹
% 储存参考轨迹,包含了参考x,y,phi
ref_pos = zeros(3,n);
ref_pos(1,:) = target{2}; % X坐标
ref_pos(2,:) = target{3}; % Y坐标
% 储存曲线曲率
ref_k = zeros(1,n);
% 参考速度tt_v
ref_v = target{4}; 
% 获取差量
dx = zeros(1,n);
dy = zeros(1,n);
dx(1,1:n - 1) = ref_pos(1,2:n) - ref_pos(1,1:n-1);
dy(1,1:n - 1) = ref_pos(2,2:n) - ref_pos(2,1:n-1);
dx(1,n) = dx(1,n - 1); % 保证导数连续
dy(1,n) = dy(1,n - 1); % 保证导数连续
% 获取参考phi
ref_pos(3,1:n) = arctan(dy,dx);

% 计算二阶导数
ddx = zeros(1,n);
ddy = zeros(1,n);
ddx(1,2:n - 1) = dx(1,2:n - 1) - dx(1,1:n - 2);
ddy(1,2:n - 1) = dy(1,2:n - 1) - dy(1,1:n - 2);
% 保证连续
ddx(1,1) = ddx(1,2);
ddx(1,n) = ddx(1,n - 1);
ddy(1,1) = ddy(1,2);
ddy(1,n) = ddy(1,n - 1);
% 获得曲线曲率
ref_k = (ddy .* dx - ddx .* dy) ./ (dx .^ 2 + dy .^ 2).^(1.5); 

%% 相关参数默认初始化
% 最大误差
if MaxErr == -1
    maxDistance = (act_pos(1,1) - ref_pos(1,:)).^2 + (act_pos(2,1) - ref_pos(2,:)).^2;
    [MaxErr,~] = max(maxDistance);
end
% 控制量约束
if isempty(Urange)
    % 控制量最小值和最大值
    umin = [-0.2;-0.54];
    umax = [0.2;0.332];
    % 控制量变化量的最大值和最小值
    delta_u_min = [-0.05;-0.64];
    delta_u_max = [0.05;0.64];
    Urange = {umin,umax,delta_u_min,delta_u_max};
end
ERR = 0;

%% MPC控制
index = 0; % 储存跟踪点
i = 1; % 储存当前点的下标
u_pre = [0;0]; % 上一次控制量和参考量的误差值
% 开始跟踪
while index < n 
    % 根据当前位置寻找参考曲线上最近的点
    distance = (ref_pos(1,:) - act_pos(1,i)).^2 + (ref_pos(2,:) - act_pos(2,i)).^2;
    [error,index] = min(distance);
    if error >= MaxErr
        % 代表输入未正确
        ERR = -1;
        break;
    end

    % 计算参考ref_delta_f
    ref_delta_f = arctan(l,1 / ref_k(index));
    
    % 滚动优化
    [u_pre] = MPC_OPTI(act_pos(:,i),u_pre,ref_pos,index,[ref_v;ref_delta_f],dt,l,Urange);
    
    % 更新状态
    act_pos(1,i + 1) = act_pos(1,i) + u_pre(1) * cos(act_pos(3,i)) * dt; % x
    act_pos(2,i + 1) = act_pos(2,i) + u_pre(1) * sin(act_pos(3,i)) * dt; % y
    act_pos(3,i + 1) = act_pos(3,i) + u_pre(1) / l * tan(u_pre(2)) * dt; % phi
    % 将phi转换到0到2pi
    act_pos(3,i + 1) = arc2pi(act_pos(3,i + 1));
    v = u_pre(1); % v
    act_ctl(:,i + 1) = u_pre;
    
    % 下一次循环的u_pre
    u_pre = u_pre - [ref_v;ref_delta_f];
    i = i + 1;
end
%% 返回值
x = act_pos(1,1:i);
y = act_pos(2,1:i);
phi = act_pos(3,1:i);
v = act_ctl(1,1:i);
delta_f = act_ctl(2,1:i);
end

