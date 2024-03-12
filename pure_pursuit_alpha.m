clear var
clear all
clc

A = readmatrix("path1.csv");% 파일 로드
plot(A(:,1),A(:,2),'.')
hold on
c_num = length(A(:,1));

x_c = 402470.5; %임의의 차량의 x 좌표
y_c = 4132970.6; %임의의 차량의 y 좌표
l_d = 3; %ahead_distance(m)
theta = 60/180*pi %head_angle(degree)

plot(x_c, y_c, '.')
hold on
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 거리 구하는 코드, 에러 구하는 코드
for n = 1:1:c_num%행 만큼 반복
    dis(n,1) = sqrt((x_c - A(n,1))^2+(y_c - A(n,2))^2);
    err(n,1) = abs(l_d - dis(n,1));
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 가장 l_d와 가까운 점을 찾아 x_r, y_r로 설정(r: 에러가 가장 작은 인덱스)
r = find(err == min(err))
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

x_r = A(r,1); % path 위의 l_d에 해당하는 x 좌표
y_r = A(r,2); % path 위의 l_d에 해당하는 y 좌표

plot([x_c x_r], [y_c y_r]) % l_d의 그래프
hold on
plot([x_c x_c+50*cos(theta)], [y_c y_c+50*sin(theta)]) % 설정한 각도와 좌표축이 이루는 각의 모습 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% a 각 구하기
a = (atan2((y_r - y_c) , (x_r - x_c)) - theta)/pi*180 % a 값의 degree 값
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

