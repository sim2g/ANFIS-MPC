clc;
clear;
dt = xlsread('plot21.xlsx');
dt2 = xlsread('plot22.xlsx');
data = xlsread('State 21.xlsx');
data2 = xlsread('State 22.xlsx');

T = 1;
N  = 100;
X_out = zeros(N,2);
T_out = zeros(N,1);

for k=1:1:N
    X_out(k,1) = k*T;
    X_out(k,2)=0.005*(k*T)^2;
    X_out(k,3)=atan(0.01*k*T);
    T_out(k,1)=k*T;
end

figure(1)
plot(X_out(1:N,1),X_out(1:N,2))
xlabel('Position X')
ylabel('Position Y')
title('Linear-MPC result trajectory for case 2')
hold on
plot(dt(11661:44472,2),dt(11661:44472,3))
legend('Reference track','Vehicle simualtion trajectory');

figure(2)
plot(X_out(1:N,1),X_out(1:N,2))
xlabel('Position X')
ylabel('Position Y')
title('ANFIS-MPC result trajectory for case 2')
hold on
plot(dt2(10707:44683,2),dt2(10707:44683,3))
legend('Reference track','Vehicle simualtion trajectory');

figure(3)
plot(X_out(1:N,1),X_out(1:N,3))
xlabel('Position X')
ylabel('Rotation angle')
title('Linear-MPC result rotation for case 2')
ylim([0 0.9])
hold on
plot(data(2182:35816,4),data(2182:35816,6))
legend('Reference rotation','Actual vehicle rotation');

figure(4)
plot(X_out(1:N,1),X_out(1:N,3))
xlabel('Position X')
ylabel('Rotation angle')
title('ANFIS-MPC result rotation for case 2')
ylim([0 0.9])
hold on
plot(data2(907:34083,4),data2(907:34083,6))
legend('Reference rotation','Actual vehicle rotation');





figure(5)
y_error=zeros(35816-2182,1);
for i=2182:1:35816
    y_error(i-2181,1)=data(i,5)-0.005*(data(i,4))^2;
end
plot(data(2182:35816,4),y_error,'color','b')
xlabel('Position X')
ylabel('Error in position Y')
title('Result trajectory erorr plot for case 2')
hold on
y_error1=zeros(34083-907,1);
for i=907:1:34083
    y_error1(i-906,1)=data2(i,5)-0.005*(data2(i,4))^2;
end
plot(data2(907:34083,4),y_error1,'color','r')
legend('Linear MPC','ANFIS MPC');


rmse_linear_T = sqrt(mean((y_error).^2));
rmse_ANFIS_T = sqrt(mean((y_error1).^2));


figure(6)
y_error=zeros(35816-2182,1);
for i=2182:1:35816
    y_error(i-2181,1)=data(i,2)-atan(0.01*data(i,4));
end
plot(data(2182:35816,4),y_error,'color','b')
xlabel('Position X')
ylabel('Error in Rotation')
title('Result Rotation erorr plot for case 2')
%ylim([-0.14 0])
hold on
y_error1=zeros(34083-907,1);
for i=907:1:34083
    y_error1(i-906,1)=data2(i,2)-atan(0.01*data2(i,4));
end
plot(data2(907:34083,4),y_error1,'color','r')
legend('Linear MPC','ANFIS MPC');

rmse_linear_R = sqrt(mean((y_error).^2));
rmse_ANFIS_R = sqrt(mean((y_error1).^2));