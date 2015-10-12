function Points = points(number)
up = 480;
low = -480;
K = rand(number, 3);
K = K * (up - low);
K = K + low;
Points = [K ones(number,1)];
end

