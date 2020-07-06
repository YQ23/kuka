clear;
clc;
close all;

%计算齐次变换矩阵A,输入参数顺序为(theta,d,a,alpha)
A1 = cal_A(150,0.34,0,-90);
A2 = cal_A(-60,0,0,90);
A3 = cal_A(120,0.4,0,-90);
A4 = cal_A(60,0,0,90);
A5 = cal_A(90,0.4,0,-90);
A6 = cal_A(-60,0,0,90);
A7 = cal_A(-120,0.126,0,0);


% A1 = cal_A(0,0.34,0,-90);
% A2 = cal_A(0,0,0,90);
% A3 = cal_A(0,0.4,0,-90);
% A4 = cal_A(0,0,0,90);
% A5 = cal_A(0,0.4,0,-90);
% A6 = cal_A(0,0,0,90);
% A7 = cal_A(0,0.126,0,0);


%计算T矩阵
T1 = A1;
T2 = A1*A2;
T3 = A1*A2*A3;
T4 = A1*A2*A3*A4;
T5 = A1*A2*A3*A4*A5;
T6 = A1*A2*A3*A4*A5*A6;
T7 = A1*A2*A3*A4*A5*A6*A7;

%根据T矩阵得到相应的姿态角度R和坐标向量o,通过z=R*k计算得到z
[R1,o1,z1] = cal_T(T1);
[R2,o2,z2] = cal_T(T2);
[R3,o3,z3] = cal_T(T3);
[R4,o4,z4] = cal_T(T4);
[R5,o5,z5] = cal_T(T5);
[R6,o6,z6] = cal_T(T6);
[R7,o7,z7] = cal_T(T7);

%计算zi×(on-oi),得到雅克比矩阵上面部分
z0 = [0;0;1];o0 = [0;0;0];
h0 = cal_zo(z0,o7,o0);
h1 = cal_zo(z1,o7,o1);
h2 = cal_zo(z2,o7,o2);
h3 = cal_zo(z3,o7,o3);
h4 = cal_zo(z4,o7,o4);
h5 = cal_zo(z5,o7,o5);
h6 = cal_zo(z6,o7,o6);
J = zeros(6,7);
J(1:3,1) = h0;
J(1:3,2) = h1;
J(1:3,3) = h2;
J(1:3,4) = h3;
J(1:3,5) = h4;
J(1:3,6) = h5;
J(1:3,7) = h6;
J(4:6,1) = z0;
J(4:6,2) = z1;
J(4:6,3) = z2;
J(4:6,4) = z3;
J(4:6,5) = z4;
J(4:6,6) = z5;
J(4:6,7) = z6;

%结果展示
disp A1;disp(A1);
disp A2;disp(A2);
disp A3;disp(A3);
disp A4;disp(A4);
disp A5;disp(A5);
disp A6;disp(A6);
disp A7;disp(A7);
disp T1;disp(T1);
disp T2;disp(T2);
disp T3;disp(T3);
disp T4;disp(T4);
disp T5;disp(T5);
disp T6;disp(T6);
disp T7;disp(T7);
disp J;disp(J);

%计算zi×(on-oi)
function [result] = cal_zo(zn,o1,o0)
on = o1 - o0;
temp = zeros(2,3);
temp(1,:) = zn';
temp(2,1:3) = on';
result = zeros(3,1);
result(1) = temp(1,2).*temp(2,3) - temp(1,3).*temp(2,2);
result(2) =-( temp(1,1).*temp(2,3) - temp(1,3).*temp(2,1));
result(3) = temp(1,1).*temp(2,2) - temp(2,1).*temp(1,2);
end

%求T矩阵
function [Rn,on,zn] = cal_T(Tn)
Rn = Tn(1:3,1:3);
on = Tn(1:3,4);
k = [0;0;1];
zn = Rn * k;
end

%求A矩阵
function [A] = cal_A(theta,d,a,alpha)
rotz = eye(4);
rotz(1,1) = cos(theta.*pi./180);
rotz(1,2) = -sin(theta.*pi./180);
rotz(2,1) = sin(theta.*pi./180);
rotz(2,2) = cos(theta.*pi./180);
transz = eye(4);
transz(3,4) = d;
transx = eye(4);
transx(1,4) = a;
rotx = eye(4);
rotx(2,2) = cos(alpha.*pi./180);
rotx(2,3) = -sin(alpha.*pi./180);
rotx(3,2) = sin(alpha.*pi./180);
rotx(3,3) = cos(alpha.*pi./180);
A = rotz * transz * transx * rotx;
end