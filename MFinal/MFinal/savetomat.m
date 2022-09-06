clc;
clear;
Ini_data = readtable('Kuorun Interpolated.csv');
data = table2array(Ini_data);
data = data';
Vx = data(2,10:3000);
Vy = data(1,10:3000);
rotation = data(3,10:3000);
w = data(4,10:3000);
steering = data(5,10:3000);

Vx_out = data(2,11:3001);
Vy_out = data(1,11:3001);
rotation_out = data(3,11:3001);
w_out = data(4,11:3001);
steering_out = data(5,11:3001);

save traindata1.mat Vx Vy rotation w steering Vx_out Vy_out rotation_out w_out steering_out