clear
clc

% 不要用Ctrl + Enter运行
% 取消注释后F5运行

%% sin
% init_x = -0.3;
% init_y = 0;
% init_phi = 0;
% l = 2;
% init_v = 0.1;
% dt = 0.1;
% tt_x = linspace(0,100,1001);
% tt_y = sin(tt_x) / 3;
% tt_v = 1;
% init = [init_x,init_y,init_phi,l,init_v];
% target = {dt,tt_x,tt_y,tt_v};
% plot(tt_x,tt_y,'LineWidth',2);
% hold on;
% [act_x,act_y,act_phi,act_v,act_delta_f] = MPC(init,target,-1,{});
% plot(act_x,act_y,'LineWidth',2);

%% 直线
% init_x = 0;
% init_y = 0;
% init_phi = 0;
% l = 2;
% init_v = 0.1;
% dt = 0.1;
% tt_x = linspace(0,100,1001);
% tt_y = linspace(10,10,1001);
% tt_v = 1;
% init = [init_x,init_y,init_phi,l,init_v];
% target = {dt,tt_x,tt_y,tt_v};
% plot(tt_x,tt_y,'LineWidth',2);
% hold on;
% [act_x,act_y,act_phi,act_v,act_delta_f] = MPC(init,target,-1,{});
% plot(act_x,act_y,'LineWidth',2);

%% 圆
% init_x = 10.1;
% init_y = 0;
% init_phi = 0;
% l = 2;
% init_v = 0.1;
% dt = 0.1;
% r = 10;
% arc = linspace(0,2*pi - 0.01,1000); % 防止结束点和初始点重合
% tt_x = r * cos(arc);
% tt_y = r * sin(arc);
% tt_v = 1;
% init = [init_x,init_y,init_phi,l,init_v];
% target = {dt,tt_x,tt_y,tt_v};
% plot(tt_x,tt_y,'LineWidth',1);
% hold on;
% [act_x,act_y,act_phi,act_v,act_delta_f] = MPC(init,target,-1,{});
% plot(act_x,act_y,'LineWidth',1);

%% 半圆
% init_x = 5;
% init_y = -7;
% init_phi = -pi;
% l = 2;
% init_v = 0.1;
% dt = 0.1;
% r = 10;
% arc = linspace(-pi/2 + pi / 3,pi/2,1000);
% tt_x = r * cos(arc);
% tt_y = 5 + r * sin(arc);
% tt_v = 1;
% init = [init_x,init_y,init_phi,l,init_v];
% target = {dt,tt_x,tt_y,tt_v};
% plot(tt_x,tt_y,'LineWidth',2);
% hold on;
% [act_x,act_y,act_phi,act_v,act_delta_f] = MPC(init,target,-1,{});
% plot(act_x,act_y,'LineWidth',2);

%% 半圆
% init_x = 0;
% init_y = 10.01;
% init_phi = pi;
% l = 2;
% init_v = 0.1;
% dt = 0.1;
% r = 10;
% arc = linspace(pi / 2,pi * 3 / 2,1000);
% tt_x = r * cos(arc);
% tt_y = r * sin(arc);
% tt_v = 1;
% init = [init_x,init_y,init_phi,l,init_v];
% target = {dt,tt_x,tt_y,tt_v};
% plot(tt_x,tt_y,'LineWidth',1);
% hold on;
% [act_x,act_y,act_phi,act_v,act_delta_f] = MPC(init,target,-1);
% plot(act_x,act_y,'LineWidth',1);

%% 误差大于给定误差例子
% 使用圆形
% init_x = 10.1;
% init_y = 0;
% init_phi = 0;
% l = 2;
% init_v = 0.1;
% dt = 0.1;
% r = 10;
% arc = linspace(0,2*pi - 0.01,1000); % 防止结束点和初始点重合
% tt_x = r * cos(arc);
% tt_y = r * sin(arc);
% tt_v = 1;
% init = [init_x,init_y,init_phi,l,init_v];
% target = {dt,tt_x,tt_y,tt_v};
% plot(tt_x,tt_y,'LineWidth',1);
% hold on;
% [act_x,act_y,act_phi,act_v,act_delta_f,ERR] = MPC(init,target,1,{});
% if ERR == -1
%     disp('与目标轨迹误差过大')
% end
% plot(act_x,act_y,'LineWidth',1);