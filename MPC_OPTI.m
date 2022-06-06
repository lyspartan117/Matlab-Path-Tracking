function [u_pre] = MPC_OPTI(pos,u_pre,ref_pos,index,ref_u,dt,l,Urange)
%MPC_OPTI MPC滚动优化
%   当前位置pos
%   上一时刻控制量和参考量的差值u_pre
%   参考状态量ref_pos;
%   参考点下标index;
%   参考控制量ref_u = [tt_v;delta_f];
%   采样时间dt
%   车辆长度l
%   返回控制量u_pre

%% 初始化
% Nx状态量个数
Nx = length(ref_pos(:,1));
% Nu控制量个数
Nu = length(ref_u);
% Np预测时域
Np = 60;
% Nc控制时域
Nc = 30;
% Q
Q = 100 * eye(Np * Nx); % Q为(Np * Nx) × (Np * Nx)的单位矩阵
% R
R = 5 * eye(Nc * Nu); % R为=(Nc * Nu) × (Nc * Nu)的单位矩阵
% 松弛因子
row = 10;

% 控制量约束
% 控制量最小值和最大值
umin = Urange{1};
umax = Urange{2};
% 控制量变化量的最大值和最小值
delta_u_min = Urange{3};
delta_u_max = Urange{4};

% a,b矩阵
a = [1     0     -dt * ref_u(1) * sin(ref_pos(3,index));
     0     1      dt * ref_u(1) * cos(ref_pos(3,index));
     0     0                        1                  ]; % Nx * Nx
b = [dt * cos(ref_pos(3,index))                          0;
     dt * sin(ref_pos(3,index))                          0;
     dt * tan(ref_pos(3,index)) / l      dt * ref_u(1) / l / cos(ref_u(2))^2]; % Nx * Nu
 
%% 滚动优化所需矩阵
ksi = [pos - ref_pos(:,index);u_pre]; % (Nx + Nu) * 1

% A,B矩阵
A = cell(2,2);
A{1,1} = a;
A{1,2} = b;
A{2,1} = zeros(Nu,Nx);
A{2,2} = eye(Nu,Nu);
A = cell2mat(A); % (Nx + Nu) * (Nx + Nu)

B = cell(2,1);
B{1,1} = b;
B{2,1} = eye(Nu,Nu);
B = cell2mat(B); % (Nx + Nu) * Nu

% C矩阵
C = [eye(Nx),zeros(Nx,Nu)]; % Nx × (Nx + Nu)

% PHI矩阵
PHI = cell(Np,1);
for i = 1:1:Np
    PHI{i,1} = C * A^i;
end
PHI = cell2mat(PHI); % (Np * Nx) * (Nx + Nu)

% THETA矩阵
THETA = cell(Np,Nc);
for i = 1:1:Np
    for j = 1:1:Nc
        if j <= i
            THETA{i,j} = C * A^(i - j) * B;
        else
            THETA{i,j} = zeros(Nx,Nu);
        end
    end
end
THETA = cell2mat(THETA); % (Nx * Np) * (Nc * Nu)

%% 二次规划矩阵
% H矩阵
H = cell(2,2);
H{1,1} = THETA' * Q * THETA + R;
H{1,2} = zeros(Nc * Nu,1);
H{2,1} = zeros(1,Nc * Nu);
H{2,2} = row;
H = cell2mat(H); % (Nc * Nu + 1) × (Nc * Nu + 1);

% E矩阵
E = PHI * ksi; % (Nx * Np) × 1

% g矩阵
g = cell(1,2);
g{1,1} = E' * Q * THETA;
g{1,2} = 0;
g = cell2mat(g); % 1 × (Nc + Nu + 1)

%% 约束矩阵
% A_I矩阵
A_I = tril(ones(Nc));
A_I = kron(A_I,eye(Nu)); % (Nc * Nu) * (Nc * Nu)

% Ut矩阵
Ut = kron(ones(Nc,1),u_pre); % (Nc * Nu) * 1

% 约束矩阵
Umin = kron(ones(Nc,1),umin);
Umax = kron(ones(Nc,1),umax);
delta_U_min = kron(ones(Nc,1),delta_u_min);
delta_U_max = kron(ones(Nc,1),delta_u_max);

% quadprog函数中Ax <= b中矩阵A
A_qp = { A_I , zeros(Nc * Nu,1);
        -A_I , zeros(Nc * Nu,1)};
A_qp = cell2mat(A_qp);
    
% quadprog函数中Ax <= b中矩阵b
b_qp = { Umax - Ut;
        -Umin + Ut};
b_qp = cell2mat(b_qp);

% ΔU上下边界
lb = [delta_U_min;0];
ub = [delta_U_max;1];

%% 二次规划求解
% options = optimoptions('quadprog','MaxIterations',100,'TolFun',1e-16);
options = optimset('Algorithm','interior-point-convex','Display','off');
delta_U = quadprog(H,g,A_qp,b_qp,[],[],lb,ub,[],options); % (Nu * Nc + 1)

%% 返回值
% 控制量和参考值之间的误差
u_pre = delta_U(1:2,1) + u_pre;

% 实际控制量
u_pre = ref_u + u_pre;
end

