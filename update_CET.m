function [next_dx,next_dy,next_v,next_phi] = update_CET(v,phi,a,delta_f,dt,l)
%UPDATE_CET 根据当前状态计算下一时刻的位置相关参数，仅限于CET方法
%   输入当前速度v,当前航向角phi,加速度a,时间间隔dt,车辆长度l
next_phi = phi + v * tan(delta_f) / l * dt; % 先应该改变前轮转向角
next_dx = v * cos(next_phi) * dt;
next_dy = v * sin(next_phi) * dt;
next_v = v + a * dt;
end

