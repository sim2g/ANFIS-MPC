clc;
clear;
dt = xlsread('plot11.xlsx');
dt2 = xlsread('plot12.xlsx');
data = xlsread('State 31.xlsx');
data2 = xlsread('State 32.xlsx');

shape=2.4;%参数名称，用于参考轨迹生成
dx1=25;dx2=21.95;%没有任何实际意义，只是参数名称
dy1=54.05;dy2=5.7;%没有任何实际意义，只是参数名称
Xs1=27.19;Xs2=56.46;%参数名称
T = 1;
N  = 100;
X_out = zeros(N,2);
T_out = zeros(N,1);
for k=1:1:N
    X_out(k,1) = k*T;
    z1=shape/dx1*(k*T-Xs1)-shape/2;
    z2=shape/dx2*(k*T-Xs2)-shape/2;
    X_out(k,2)=dy1/2*(1+tanh(z1))-dy2/2*(1+tanh(z2));
    X_out(k,3)=atan(dy1*(1/cosh(z1))^2*(1.2/dx1)-dy2*(1/cosh(z2))^2*(1.2/dx2));
    T_out(k,1)=k*T;
end
figure(1)
plot(X_out(1:N,1),X_out(1:N,2))
xlabel('Position X')
ylabel('Position Y')
ylim([-10 60])
title('MPC controller A and B tracking performance for difference parameters')
%title('Linear-MPC result trajectory for case 3')
hold on
plot(data(2139:41212,4),data(2139:41212,5))
hold on
plot(dt(11383:49430,2),dt(11383:49430,3))

%legend('Reference track','Vehicle simualtion trajectory');
legend('Reference track','Controller A','Controller B');


figure(2)
plot(X_out(1:N,1),X_out(1:N,2))
xlabel('Position X')
ylabel('Position Y')
ylim([-10 60])
title('ANFIS-MPC result trajectory for case 3')
hold on
plot(dt2(2483:41828,2),dt2(2483:41828,3))
legend('Reference track','Vehicle simualtion trajectory');

figure(3)
plot(X_out(1:N,1),X_out(1:N,3))
xlabel('Position X')
ylabel('Rotation angle')
title('Linear-MPC result rotation for case 3')
ylim([-1.5 1.5])
hold on
plot(data(2139:41212,4),data(2139:41212,6))
legend('Reference rotation','Actual vehicle rotation');

figure(4)
plot(X_out(1:N,1),X_out(1:N,3))
xlabel('Position X')
ylabel('Rotation angle')
title('ANFIS-MPC result rotation for case 3')
ylim([-1.5 1.5])
hold on
plot(data2(275:39018,4),data2(275:39018,6))
legend('Reference rotation','Actual vehicle rotation');





figure(5)
y_error=zeros(41212-2139,1);
for i=2139:1:41212
    z1=shape/dx1*(data(i,4)-Xs1)-shape/2;
    z2=shape/dx2*(data(i,4)-Xs2)-shape/2;
    y_error(i-2138,1)=data(i,5)-(dy1/2*(1+tanh(z1))-dy2/2*(1+tanh(z2)));
end
plot(data(2139:41212,4),y_error,'color','b')
xlabel('Position X')
ylabel('Error in position Y')
title('Result trajectory erorr plot for case 3')
hold on
y_error1=zeros(39018-275,1);
for i=275:1:39018
    z1=shape/dx1*(data2(i,4)-Xs1)-shape/2;
    z2=shape/dx2*(data2(i,4)-Xs2)-shape/2;
    y_error1(i-274,1)=data2(i,5)-(dy1/2*(1+tanh(z1))-dy2/2*(1+tanh(z2)));
end
plot(data2(275:39018,4),y_error1,'color','r')
legend('Linear MPC','ANFIS MPC');

rmse_linear_T = sqrt(mean((y_error).^2));
rmse_ANFIS_T = sqrt(mean((y_error1).^2));


figure(6)
y_error=zeros(41212-2139,1);
for i=2139:1:41212
    z1=shape/dx1*(data(i,4)-Xs1)-shape/2;
    z2=shape/dx2*(data(i,4)-Xs2)-shape/2;
    y_error(i-2138,1)=data(i,2)-atan(dy1*(1/cosh(z1))^2*(1.2/dx1)-dy2*(1/cosh(z2))^2*(1.2/dx2));
end
plot(data(2139:41212,4),y_error,'color','b')
xlabel('Position X')
ylabel('Error in Rotation')
title('Result Rotation erorr plot for case 3')
%ylim([-0.14 0])
hold on
y_error1=zeros(39018-275,1);
for i=275:1:39018
    z1=shape/dx1*(data2(i,4)-Xs1)-shape/2;
    z2=shape/dx2*(data2(i,4)-Xs2)-shape/2;
    y_error1(i-274,1)=data2(i,2)-atan(dy1*(1/cosh(z1))^2*(1.2/dx1)-dy2*(1/cosh(z2))^2*(1.2/dx2));
end
plot(data2(275:39018,4),y_error1,'color','r')
legend('Linear MPC','ANFIS MPC');

rmse_linear_R = sqrt(mean((y_error).^2));
rmse_ANFIS_R = sqrt(mean((y_error1).^2));