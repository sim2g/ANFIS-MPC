clc;
clear;
%data = xlsread('plot31.xlsx');
%data2 = xlsread('plot32.xlsx');
data = xlsread('State 41.xlsx');
data2 = xlsread('State 42.xlsx');

T = 1;
N  = 110;
X_out = zeros(N,2);
T_out = zeros(N,1);

for k=1:1:N
    X_out(k,1) = k*T;
    X_out(k,2)=(6859*(k*T)^3*(20577*(k*T)^2-3781760*(k*T)+175110000))/1000000000000000;
    X_out(k,3)=atan((6859*(k*T)^2*(20577*(k*T)^2-3025408*(k*T)+105066000))/200000000000000);
    T_out(k,1)=k*T;
end

figure(1)
plot(X_out(1:N,1),X_out(1:N,2))
xlabel('Position X')
ylabel('Position Y')
ylim([-10 80])
title('Linear-MPC result trajectory for case 4')
hold on
plot(data(708:55111,2),data(708:55111,3))
legend('Reference track','Vehicle simualtion trajectory');

figure(2)
plot(X_out(1:N,1),X_out(1:N,2))
xlabel('Position X')
ylabel('Position Y')
ylim([-10 80])
title('ANFIS-MPC result trajectory for case 4')
hold on
plot(data2(136:54362,2),data2(136:54362,3))
legend('Reference track','Vehicle simualtion trajectory');

figure(3)
plot(X_out(1:N,1),X_out(1:N,3))
xlabel('Position X')
ylabel('Rotation angle')
title('Linear-MPC result rotation for case 4')
ylim([-1.5 1.5])
hold on
plot(data(264:54290,4),data(264:54290,6))
legend('Reference rotation','Actual vehicle rotation');

figure(4)
plot(X_out(1:N,1),X_out(1:N,3))
xlabel('Position X')
ylabel('Rotation angle')
title('ANFIS-MPC result rotation for case 4')
ylim([-1.5 1.5])
hold on
plot(data2(3780:58188,4),data2(3780:58188,6))
legend('Reference rotation','Actual vehicle rotation');





figure(5)
y_error=zeros(54290-264,1);
for i=264:1:54290
    y_error(i-263,1)=data(i,5)-((6859*(data(i,4))^3*(20577*(data(i,4))^2-3781760*(data(i,4))+175110000))/1000000000000000);
end
plot(data(264:54290,4),y_error,'color','b')
xlabel('Position X')
ylabel('Error in position Y')
title('Result trajectory erorr plot for case 4')
hold on
y_error1=zeros(58188-3780,1);
for i=3780:1:58188
    y_error1(i-3779,1)=data2(i,5)-((6859*(data2(i,4))^3*(20577*(data2(i,4))^2-3781760*(data2(i,4))+175110000))/1000000000000000);
end
plot(data2(3780:58188,4),y_error1,'color','r')
legend('Linear MPC','ANFIS MPC');

rmse_linear_T = sqrt(mean((y_error).^2));
rmse_ANFIS_T = sqrt(mean((y_error1).^2));


figure(6)
y_error=zeros(54290-264,1);
for i=264:1:54290
    y_error(i-263,1)=data(i,2)-atan((6859*(data(i,4))^2*(20577*(data(i,4))^2-3025408*(data(i,4))+105066000))/200000000000000);
end
plot(data(264:54290,4),y_error,'color','b')
xlabel('Position X')
ylabel('Error in Rotation')
title('Result Rotation erorr plot for case 4')
%ylim([-0.14 0])
hold on
y_error1=zeros(58188-3780,1);
for i=3780:1:58188
    y_error1(i-3779,1)=data2(i,2)-atan((6859*(data2(i,4))^2*(20577*(data2(i,4))^2-3025408*(data2(i,4))+105066000))/200000000000000);
end
plot(data2(3780:58188,4),y_error1,'color','r')
legend('Linear MPC','ANFIS MPC');

rmse_linear_R = sqrt(mean((y_error).^2));
rmse_ANFIS_R = sqrt(mean((y_error1).^2));