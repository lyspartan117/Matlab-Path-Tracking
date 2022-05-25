function [error] = calcERR_CET(pos1,pos2,phi,pos3)
%calcERR_CET CET模型下计算累计误差
%   输入pos2为目标点,pos1为当前点,均储存坐标x,y信息,再输入phi航偏角信息,pos1和pos2以向量形式输入
%   pos3为pos2下一个点，用以判断pos1和直线的位置关系
ld = pos2 - pos1;
if(ld(2) == 0) % 防止出现0/0
    arc = 0;
else
    arc = atan(ld(2)/ld(1));
end
error = sqrt(sum(ld.^2)) * abs(sin(arc - phi));
direct = pos3 - pos2;
if ld(1) * direct(2) - ld(2) * direct(1) > 0
    error = -error;
end
end