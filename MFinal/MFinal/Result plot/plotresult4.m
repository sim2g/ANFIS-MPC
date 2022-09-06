clc;
clear;
data = xlsread('State 11.xlsx');
data2 = readmatrix('State 12.xlsx');

T = 1;
N  = 100;
X_out = zeros(N,2);
T_out = zeros(N,1);

for k=1:1:N
    X_out(k,1) = k*T;
    X_out(k,2)=0.123*k*T;
    X_out(k,3)=atan(0.123);
    T_out(k,1)=k*T;
end

figure(1)
plot(X_out(1:N,1),X_out(1:N,2))
xlabel('Position X')
ylabel('Position Y')
title('Linear-MPC result trajectory for case 1')
hold on
plot(data(28:27829,4),data(28:27829,5))
legend('Reference track','Vehicle simualtion trajectory');

figure(2)
plot(X_out(1:N,1),X_out(1:N,2))
xlabel('Position X')
ylabel('Position Y')
title('ANFIS-MPC result trajectory for case 1')
hold on
plot(data2(110:28554,4),data2(110:28554,5))
legend('Reference track','Vehicle simualtion trajectory');

figure(3)
plot(X_out(1:N,1),X_out(1:N,3))
xlabel('Position X')
ylabel('Rotation angle')
title('Linear-MPC result rotation for case 1')
ylim([0 0.2])
hold on
plot(data(28:27829,4),data(28:27829,6))
legend('Reference rotation','Actual vehicle rotation');

figure(4)
plot(X_out(1:N,1),X_out(1:N,3))
xlabel('Position X')
ylabel('Rotation angle')
title('ANFIS-MPC result rotation for case 1')
ylim([0 0.2])
hold on
plot(data2(110:28554,4),data2(110:28554,6))
legend('Reference rotation','Actual vehicle rotation');

figure(5)
y_error=zeros(27829-28,1);
for i=28:1:27829
    y_error(i-27,1)=data(i,5)-0.123*data(i,4);
end
plot(data(28:27829,4),y_error,'color','b')
xlabel('Position X')
ylabel('Error in position Y')
title('Result trajectory erorr plot for case 1')
hold on
y_error1=zeros(28554-110,1);
for i=110:1:28554
    y_error1(i-109,1)=data2(i,5)-0.123*data2(i,4);
end
plot(data2(110:28554,4),y_error1,'color','r')
legend('Linear MPC','ANFIS MPC');

rmse_linear_T = sqrt(mean((y_error).^2));
rmse_ANFIS_T = sqrt(mean((y_error1).^2));



figure(6)
y_error=zeros(27829-28,1);
for i=28:1:27829
    y_error(i-27,1)=data(i,2)-atan(0.123);
end
plot(data(28:27829,4),y_error,'color','b')
xlabel('Position X')
ylabel('Error in Rotation')
title('Result Rotation erorr plot for case 1')
ylim([-0.14 0])
hold on
y_error1=zeros(28554-110,1);
for i=110:1:28554
    y_error1(i-109,1)=data2(i,2)-atan(0.123);
end
plot(data2(110:28554,4),y_error1,'color','r')
legend('Linear MPC','ANFIS MPC');

rmse_linear_R = sqrt(mean((y_error).^2));
rmse_ANFIS_R = sqrt(mean((y_error1).^2));