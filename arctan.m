function [arc] = arctan(y,x)
%ARCTAN 四象限arctan(y/x)
%   结果为[0,2 * pi)
arc = atan2(y,x);
for i = 1:1:length(y)
    if arc(i) < 0
        arc(i) = arc(i) + 2 * pi;
    end
end
end

