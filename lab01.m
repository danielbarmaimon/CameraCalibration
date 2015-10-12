%% Step 1 Define intrinsec parameters and extrinsic parameter
au = 557.0943;      % Relation between focal lenght and pixel/area
av = 712.9824;      % Relation between focal lenght and pixel/area

u0 = 326.3819;      % Principal point from upper corner in image plane
v0 = 298.6679;      % Principal point from upper corner in image plane

f = 80;             % Focal length in mm

Tx = 100;           % Traslation in mm
Ty = 0;             % Traslation in mm
Tz = 1500;          % Traslation in mm

Phix = 0.8*pi/2;    % Rotation in rad(c
Phiy = -1.8*pi/2;   % Rotation in rad
Phix1 = pi/5;       % Rotation in rad

% Euler XYX1    - Reference for rotation
% Image size    - 640 X 480

%% Step 2 Get the extrinsic and intrinsic parameters
% Calculation of extrinsic paremeters
R1 = [1 0 0;0 cos(Phix) -sin(Phix); 0 sin(Phix) cos(Phix)];
R2 = [cos(Phiy) 0 sin(Phiy); 0 1 0; -sin(Phiy) 0 cos(Phiy)];
R3 = [1 0 0;0 cos(Phix1) -sin(Phix1); 0 sin(Phix1) cos(Phix1)];

R = R3*R2*R1;         % Rotation Matrix
T = [100 0 1500 1]';  % Translation

R1 = [R; 0 0 0];

cKw = [R1 T];         % Reference of the camera with repect to the world

% Calculation of the intrinsec parameters
I = [au 0 u0 0;...    
    0 av v0 0;
    0 0 1 0];

I1 = I * cKw;        % Camera transformation
I1 = I1 / I1(3,4);  

%% Step 3 Set points in range [-480:480;-480:480;-480:480]
Points = points(6);
number = size(Points,1);
%% Step 4 Projection (they should be non-coplanar)
[iPw, iPwNorm] = projectingPoints(Points, I1);

%% Step 5 Showing points as projections
figure(1);
axis([1 640 1 480]); % Defining axes limited with the size of our image
axis manual;
hold on;
title('Projection of 3D points');
scatter(iPwNorm(1,:),iPwNorm(2,:));
figure(2);
axis([1 640 1 480]); % Defining axes limited with the size of our image
axis manual;
hold on;
subplot(1,2,1);
scatter(iPwNorm(1,:),iPwNorm(2,:));

% Sometimes the points are not projected over the image plane [1:640 1:480]
% That means that the tracer line that joins the focal point and a 3D point
% does not cross the image plane in any point defined as image plane
% (1:640, 1:480). The shorter is the focal length and the greater is the 
% size of the plane, the more probably will be that the  3D points
% were projected over the plane.
% If the points are not spread around all the plane, only the dense area
% will be properly calibrated, giving bigger inaccuracy the further these 
% dense cloud of points is from the region we want to check.

%% Step 6 Get the 3x4 matrix (step 3, step 5) by using the method of Hall
figure(2);
A = Hall(Points, iPwNorm);

%% Step 7 - Compare the matrix obteined in Step 6 to the one defined in 2
disp('The matrix we get from Hall Method is');
disp(A);
disp('The matrix we get on step 2 is');
disp(I1);

% As we do not consider the distorsion, we should get the same results
% for both matrices.

%% Step 8 - Add Gaussian noise to the 2D points and compare results
% Creating and plotting noisy points
noise = normrnd(0, 0.5,[2,6]); % 2*sigma = 1 (we want 95% of data)
noise = [noise; zeros(1,6)];
iPwNoisy = iPwNorm + noise;
hold on;
scatter(iPwNoisy(1,:),iPwNoisy(2,:));
% Using Hall's function 
C = Hall(Points, iPwNoisy);
% Displaying the matrices with and without noise
disp('The matrix we get from Hall Method, without noise is');
disp(A);
disp('The matrix we get from Hall Method, wiht noise is');
disp(C);
[iPwNewNoisy, iPwNewNoisyNorm] = projectingPoints(Points, C);

subplot(1,2,2);
scatter(iPwNorm(1,:),iPwNorm(2,:));
hold on;
scatter(iPwNewNoisyNorm(1,:),iPwNewNoisyNorm(2,:));
% Way of measure the 2D distance
[MeanDistance6 SD6 Min6 Max6] = errorD(Points, iPwNorm, iPwNewNoisyNorm)

%% Step 9 - Increasing the number of points
% 9.1. Number of points 10
points10 = points(10);
% Projecting the points without noise
[iPw10, iPw10Norm] = projectingPoints(points10, I1);      % I'll get, with ten points, the 11 unknown values

A10 = Hall(points10, iPw10Norm); 

noise10 = normrnd(0, 0.5,[2,10]); % 2*sigma = 1 (we want 95% of data)
noise10 = [noise10; zeros(1,10)];
iPw10Noisy = iPw10Norm + noise10;
% hold on;
% scatter(iPw10Noisy(1,:),iPw10Noisy(2,:));

C10 = Hall(points10, iPw10Noisy);

disp('The matrix we get from Hall Method, without noise is');
disp(A10);
disp('The matrix we get from Hall Method, with noise is');
disp(C10);

[iPw10NewNoisy, iPw10NewNoisyNorm] = projectingPoints(points10, C10);

[MeanDistance10 SD10 Min10 Max10] = errorD(points10, iPw10Norm, iPw10NewNoisyNorm)

% 9.2. Number of points 50
points50 = points(50);
% Projecting the points without noise
[iPw50, iPw50Norm] = projectingPoints(points50, I1);      % I'll get, with ten points, the 11 unknown values

A50 = Hall(points50, iPw50Norm) 

noise50 = normrnd(0, 0.5,[2,50]); % 2*sigma = 1 (we want 95% of data)
noise50 = [noise50; zeros(1,50)];
iPw50Noisy = iPw50Norm + noise50;

C50 = Hall(points50, iPw50Noisy);

disp('The matrix we get from Hall Method, without noise is');
disp(A50);
disp('The matrix we get from Hall Method, with noise is');
disp(C50);

[iPw50NewNoisy, iPw50NewNoisyNorm] = projectingPoints(points50, C50);

% Way of measure the 2D distance
[MeanDistance50 SD50 Min50 Max50] = errorD(points50, iPw50Norm, iPw50NewNoisyNorm)
%% Step 10 - Getting Intrinsics and extrinsics from Faugeras
[IFaug, EFaug] = Faugeras(Points, iPwNorm);
disp('The original intrinsec parameters are:');
I
disp('The intrinsec parameters obtained by Faugeres method are:');
IFaug
disp('The original extrinsec parameters are:');
cKw
disp('The intrinsec parameters obtained by Faugeres method are:');
EFaug

%% Step 11 -
% Range [-1,1]
noiseRange1 = normrnd(0, 0.5,[2,6]); % 2*sigma = 1 (we want 95% of data)
noiseRange1 = [noiseRange1; zeros(1,6)];
iPwNoisyRange1 = iPwNorm + noiseRange1;

[IFaugRange1, EFaugRange1] = Faugeras(Points, iPwNoisyRange1);
[MeanDistanceR1 SDR1 MinR1 MaxR1] = errorD(Points, iPwNorm, iPwNoisyRange1)

AHallR1 = Hall(Points, iPwNoisyRange1);
[iPwNewNoisyH1, iPwNewNoisyNormH1] = projectingPoints(Points, AHallR1);
[MeanDistanceHR1 SDHR1 MinHR1 MaxHR1] = errorD(Points, iPwNorm, iPwNewNoisyNormH1)

% Range [-2,2]
noiseRange2 = normrnd(0, 1,[2,6]); % 2*sigma = 2 (we want 95% of data)
noiseRange2 = [noiseRange2; zeros(1,6)];
iPwNoisyRange2 = iPwNorm + noiseRange2;
[IFaugRange2, EFaugRange2] = Faugeras(Points, iPwNoisyRange2);
[MeanDistanceR2 SDR2 MinR2 MaxR2] = errorD(Points, iPwNorm, iPwNoisyRange2)

AHallR2 = Hall(Points, iPwNoisyRange2);
[iPwNewNoisyH2, iPwNewNoisyNormH2] = projectingPoints(Points, AHallR2);
[MeanDistanceHR2 SDHR2 MinHR2 MaxHR2] = errorD(Points, iPwNorm, iPwNewNoisyNormH2)


% Range [-3,3]
noiseRange3 = normrnd(0, 1.5,[2,6]); % 2*sigma = 3 (we want 95% of data)
noiseRange3 = [noiseRange3; zeros(1,6)];
iPwNoisyRange3 = iPwNorm + noiseRange3;
[IFaugRange3, EFaugRange3] = Faugeras(Points, iPwNoisyRange3);
[MeanDistanceR3 SDR3 MinR3 MaxR3] = errorD(Points, iPwNorm, iPwNoisyRange3)

AHallR3 = Hall(Points, iPwNoisyRange3);
[iPwNewNoisyH3, iPwNewNoisyNormH3] = projectingPoints(Points, AHallR3);
[MeanDistanceHR3 SDHR3 MinHR3 MaxHR3] = errorD(Points, iPwNorm, iPwNewNoisyNormH3)

disp('FAUGERAS RESULTS');
disp('The mean of distances for range 1, 2, 3 are:');
Distances = [MeanDistanceR1 MeanDistanceR2 MeanDistanceR3]
disp('The standard deviations for range 1, 2, 3 are:');
StandardDev = [SDR1 SDR2 SDR3]
disp('The minimum of distances for range 1, 2, 3 are:');
Minima = [MinR1 MinR2 MinR3]
disp('The minimum of distances for range 1, 2, 3 are:');
Maxima = [MaxR1 MaxR2 MaxR3]

disp('HALL RESULTS');
disp('The mean of distances for range 1, 2, 3 are:');
Distances = [MeanDistanceHR1 MeanDistanceHR2 MeanDistanceHR3]
disp('The standard deviations for range 1, 2, 3 are:');
StandardDev = [SDHR1 SDHR2 SDHR3]
disp('The minimum of distances for range 1, 2, 3 are:');
Minima = [MinHR1 MinHR2 MinHR3]
disp('The minimum of distances for range 1, 2, 3 are:');
Maxima = [MaxR1 MaxR2 MaxR3]

%% Step 12
figure(3);
% Plotting coordinate systems
hold on;
grid on;
axis square;
% World 
line('XData',[0 100],'LineWidth',2,'Color', 'r','SelectionHighlight', 'on');
line('YData',[0 100],'LineWidth',2,'Color', 'g','SelectionHighlight', 'on');
line('ZData',[0 100],'LineWidth',2,'Color', 'b','SelectionHighlight', 'on');

% Camera
originC = pinv(cKw)*[0 0 0 1]';
endxC = pinv(cKw)*[100 0 0 1]';
endyC = pinv(cKw)*[0 100 0 1]';
endzC = pinv(cKw)*[0 0 100 1]'
endxC = endxC/ endxC(4,1);
endyC = endyC/ endyC(4,1);
endzC = endzC/ endzC(4,1);
endC = [endxC endyC endzC];
%hold on;

axis equal;
line('XData',[originC(1,1) endC(1,1)], 'YData',[originC(2,1) endC(2,1)], ...
     'ZData',[originC(3,1) endC(3,1)],'LineWidth',2,'Color', 'r','SelectionHighlight', 'on');
line('XData',[originC(1,1) endC(1,2)], 'YData',[originC(2,1) endC(2,2)], ...
     'ZData',[originC(3,1) endC(3,2)],'LineWidth',2,'Color', 'g','SelectionHighlight', 'on');
line('XData',[originC(1,1) endC(1,3)], 'YData',[originC(2,1) endC(2,3)], ...
     'ZData',[originC(3,1) endC(3,3)],'LineWidth',2,'Color', 'b','SelectionHighlight', 'on');
% Plotting the 3D Points
hold on;
scatter3(Points(:,1), Points(:,2), Points(:,3), 10, 'k', 'fill'); 
% Frame limits for Image Plane
P1 = [0 0 1]';       % Coordinates of image plane
P2 = [640 0 1]';
P3 = [640 480 1]';
P4 = [0 480 1]';

cP1 = pinv(I)*P1;   % Coordinates of image plane limits respect to camera
cP2 = pinv(I)*P2;
cP3 = pinv(I)*P3;
cP4 = pinv(I)*P4;

cP1 = cP1*f;%*2;    % Adjusting to the correct focal length (right now z = 1, h = 0)
cP2 = cP2*f;%*2;
cP3 = cP3*f;%*2;
cP4 = cP4*f;%*2;

cP1(4,1) = 1;   % Adjusting scaling. Positioning the plane in front to the camera
cP2(4,1) = 1;
cP3(4,1) = 1;
cP4(4,1) = 1;

iP1w = pinv(cKw)*(cP1); % Points of the frame in world coordinates
iP2w = pinv(cKw)*(cP2);
iP3w = pinv(cKw)*(cP3);
iP4w = pinv(cKw)*(cP4);
iPw = [iP1w iP2w iP3w iP4w iP1w]; % We add first one to connect it with last
plot3(iPw(1,:), iPw(2,:), iPw(3,:));%, 'b', 'fill'); 

normal = cross(iP2w(1:3,:)-iP1w(1:3,:),...
    iP4w(1:3,:)-iP1w(1:3,:));% Plotting the plane 
A = normal(1); B = normal(2); C = normal(3);
D = -dot(normal,iP2w(1:3,:));
xx = linspace(iP1w(1,1),iP2w(1,1));
zz = linspace(iP1w(3,1),iP4w(3,1));
hold on;
[X,Z] = meshgrid(xx,zz);

Y = (A * X + C * Z + D)/ (-B);
hold on;
%S1 = surf(X,Y,Z ,'FaceAlpha',0.3, 'EdgeColor','none','FaceColor',[0 0 1]);

% Plotting projections over the image plane

iPc = pinv(I)*iPwNorm;
iPc = iPc*f;         % We set real focal length (f = 80), it was 1.
iPc(4,:)=1;         % We set homogeneous coordinate to 1 (to normalize)
iPw = pinv(cKw)*iPc
scatter3(iPw(1,:), iPw(2,:), iPw(3,:), 10, 'r', 'fill');
Points = Points';

% Plotting ray trace
for i=1:size(Points,2)
    line('XData',[iPw(1,i) Points(1,i)], 'YData',[iPw(2,i) Points(2,i)], ...
     'ZData',[iPw(3,i) Points(3,i)],'LineWidth',1,'Color', 'k');
end

% Plotting extension of lines
newPoint = zeros(3,1);
size(Points);
for i=1:size(Points,2)
    vec = [iPw(1,i)-Points(1,i), iPw(2,i)-Points(2,i), iPw(3,i)-Points(3,i)]'
    newPoint = iPw(1:3,i) + 0.4*vec
    line('XData',[iPw(1,i) newPoint(1,1)], 'YData',[iPw(2,i) newPoint(2,1)], ...
     'ZData',[iPw(3,i) newPoint(3,1)],'LineWidth',1,'Color', 'k');
end