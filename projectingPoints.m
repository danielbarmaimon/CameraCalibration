function [iPw, iPwnorm] = projectingPoints(Points, I1)
% Projecting and normalizing points over the image plane
iPw = I1 * Points';      % I'll get, with six points, the 11 unknown values
iPwnorm = iPw;
number = size(Points, 1);
for i=1:number
    iPwnorm(:,i) = iPwnorm(:,i)/iPwnorm(3,i);   % Divided by scale factor to normalize
end
end

