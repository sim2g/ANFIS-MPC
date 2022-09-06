clc;
clear;
Ini_data = readtable('Kuorun Interpolated.csv');
data = table2array(Ini_data);
data = data';

Vx = data(2,3000:4000);
Vy = data(1,3000:4000);
rotation = data(3,3000:4000);
w = data(4,3000:4000);
steering = data(5,3000:4000);

Vx_out = data(2,3001:4001);
Vy_out = data(1,3001:4001);
rotation_out = data(3,3001:4001);
w_out = data(4,3001:4001);
steering_out = data(5,3001:4001);

test_data_Vy = [Vy;Vx;rotation;w;steering;Vy_out];
test_data_Vx = [Vy;Vx;rotation;w;steering;Vx_out];
test_data_rotation = [Vy;Vx;rotation;w;steering;rotation_out];
test_data_w = [Vy;Vx;rotation;w;steering;w_out];

save testdata1.mat test_data_Vy test_data_Vx test_data_rotation test_data_w