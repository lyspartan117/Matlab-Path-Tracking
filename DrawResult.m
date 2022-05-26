function [] = DrawResult(x,y,phi,delta_f,l,tt_x,tt_y)
%DRAWRESULT 画出结果
%   输入x,y,phi,delta_f,l
%   画出图像
hold off;
direct_length = 1;
% 确认坐标轴的边界
xlbound = min(x);
xrbound = max(x);
ylbound = min(y);
yubound = max(y);
bound = [xlbound - l,xrbound + 1,ylbound - l,yubound + 1];
for i = 1:1:length(x)
    % 画目标图像
    plot(tt_x,tt_y,'b-','LineWidth',2),axis(bound);
    hold on;
    % 画汽车状态
    x_temp = x(i) + l * cos(phi(i));
    y_temp = y(i) + l * sin(phi(i));
    plot([x(i),x_temp],[y(i),y_temp],'LineWidth',3),axis(bound);
    % 画方向角箭头
    plot([x_temp,x_temp + direct_length * cos(delta_f(i) + phi(i))],[y_temp,y_temp + direct_length * sin(delta_f(i) + phi(i))],'LineWidth',2),axis(bound);
    pause(0.1);
    hold off;
end

