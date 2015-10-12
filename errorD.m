function [ MeanDistance StandardDeviation Min Max] = errorD(points, iPwNorm, iPwNewNoisyNorm)
MeanDistance = 0;
number = size(points,1);
distances = zeros(number,1);
for i = 1 : number
    t  = iPwNorm(:,i)- iPwNewNoisyNorm(:,i);
    distances(i,1) = norm(t);
    MeanDistance = MeanDistance + distances(i,1);
end
disp('The mean of distances computed using 10 points is');
MeanDistance =double(MeanDistance) / number;
Min = min(distances);
Max = max(distances);
StandardDeviation = 0;
sum = 0;
for i = 1 : number
    sum  = sum + (MeanDistance - distances(i,1))^2;
end
StandardDeviation = sqrt(sum)/number;
end

