function pos = close_point_CET(pos1,targetx,targety)
%CLOSE_POINT_CET 寻找(targetx,targety)中离pos1最近的点,返回该点对应下标
%   思路：计算每个(targetx,targety)中离pos1的距离，分为两种情况
%   距离均递增：取第一个递增点
%   距离先递减后递增：取最小值

% 如果输入只有一个点，则退出
if length(targetx) <= 1
    pos = 1;
    return
end
% 假定第一个为最小值
min = [targetx(1),targety(1)] - pos1;
min = sum(min.^2);
for pos = 2:1:length(targetx)
    temp = [targetx(pos),targety(pos)] - pos1;
    temp = sum(temp.^2);
    % 判断是否小于min
    if(temp > min)
        % 大于min则退出循环
        pos = pos - 1;
        return;
    end
    % 更新min
    min = temp;
end
% 如果找不到点了，则返回1
pos = 1;
end

