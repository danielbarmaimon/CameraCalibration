function [IFaug, EFaug] = Faugeras( Points, iPwNorm )
A = zeros (12,1);
A(12,1) = 1;    % Approximation done to avoid infinite solutions depending
                % on scale factor
number = size(Points, 1);

% But as we know that A(3,4) = 1, we can remove last term of each matrix
Q = zeros(number*2,11);
B = zeros(number*2,1);
for i=1:number
    Q((1+(2*(i-1))),:)=...
        [Points(i,1:3)  -(iPwNorm(1,i)*Points(i,1)) ...
        -(iPwNorm(1,i)*Points(i,2)) -(iPwNorm(1,i)*Points(i,3))...
        zeros(1,3) 1 0];
    Q(2*i,:)=...
        [zeros(1,3) -(iPwNorm(2,i)*Points(i,1)) ...
        -(iPwNorm(2,i)*Points(i,2)) -(iPwNorm(2,i)*Points(i,3)) ...
        Points(i,1:3)  0 1];
end

for i=1:(size(B,1)/2)
    B(1+(2*(i-1)),:)=iPwNorm(1,i);
    B(2*i,:)=iPwNorm(2,i);
end

AA = pinv(Q)*B;
%AA = pinv(Q'*Q)*Q'*B;

T1 = AA(1:3,1)';
T2 = AA(4:6,1)';
T3 = AA(7:9,1)';
C1 = AA(10,1);
C2 = AA(11,1);

% Calculating the intrinsics parameters of camera
u0 = (T1*T2')/((norm(T2))^2);
v0 = (T2*T3')/((norm(T2))^2);
au = norm(cross(T1',T2'))/((norm(T2))^2);
av = norm(cross(T2',T3'))/((norm(T2))^2);

IFaug = [au 0 u0 0;...    
    0 av v0 0;
    0 0 1 0];
% Calculating the extrinsics parameters of camera
r1 = (norm(T2)/norm(cross(T1',T2')))*(T1-((T1*T2')/(norm(T2)^2))*T2);
r2 = (norm(T2)/norm(cross(T2',T3')))*(T3-((T2*T3')/(norm(T2)^2))*T2);
r3 = T2/norm(T2);

tx = (norm(T2)/norm(cross(T1',T2')))*(C1-((T1*T2')/(norm(T2)^2)));
ty = (norm(T2)/norm(cross(T2',T3')))*(C2-((T2*T3')/(norm(T2)^2)));
tz = 1 / norm(T2);

EFaug = [r1 tx; r2 ty; r3 tz; 0 0 0 1];
end