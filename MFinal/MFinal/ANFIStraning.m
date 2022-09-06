clc;
clear;

load traindata1.mat
load testdata1.mat

%--------------------------------------------------------------------------
data1  = [Vy;Vx;rotation;w;steering;Vy_out];

trnData=data1';
numMFs=2;
mfType='gbellmf';
in_fis = genfis1(trnData,numMFs,mfType);

[out_fis1,~,stepSize1] = anfis(trnData,in_fis,300);

figure;
plot(stepSize1);
grid on
xlabel('Iteration times');
ylabel('ANFIS training error');
title('Sub-system Vy case');

figure;
plotfis(in_fis);
title('Input-Output scheme for the sub-system Vy case');

figure;
plot(test_data_Vy(6,:),'b');
hold on
plot(evalfis(test_data_Vy([1 2 3 4 5],:)',out_fis1),'r');
ylabel('Velocity y');
xlabel('Time step');
legend('Vehicle output','ANFIS prediction output');
title('Sub-system Vy case');



%--------------------------------------------------------------------------

data2  = [Vy;Vx;rotation;w;steering;Vx_out];

trnData=data2';
numMFs=2;
mfType='gbellmf';
in_fis = genfis1(trnData,numMFs,mfType);

[out_fis2,~,stepSize2] = anfis(trnData,in_fis,300);

figure;
plot(stepSize2);
grid on
xlabel('Iteration times');
ylabel('Anfis training error');
title('Sub-system Vx case');

figure;
plotfis(in_fis);
title('Input-Output scheme for the sub-system Vx case');

figure;
plot(test_data_Vx(6,:),'b');
hold on
plot(evalfis(test_data_Vx([1 2 3 4 5],:)',out_fis2),'r');
ylabel('Velocity x');
xlabel('Time step');
legend('Vehicle output','ANFIS prediction output');
title('Sub-system Vx case');



%--------------------------------------------------------------------------
data3  = [Vy;Vx;rotation;w;steering;rotation_out];

trnData=data3';
numMFs=2;
mfType='gbellmf';
in_fis = genfis1(trnData,numMFs,mfType);

[out_fis3,~,stepSize3] = anfis(trnData,in_fis,300);

figure;
plot(stepSize3);
grid on
xlabel('Iteration times');
ylabel('Anfis training error');
title('Sub-system ROTATION case');

figure;
plotfis(in_fis);
title('Input-Output scheme for the sub-system ROTATION case');

figure;
plot(test_data_rotation(6,:),'b');
hold on
plot(evalfis(test_data_rotation([1 2 3 4 5],:)',out_fis3),'r');
ylabel('Rotation angle');
xlabel('Time step');
legend('Vehicle output','ANFIS prediction output');
title('Sub-system ROTATION case');


%--------------------------------------------------------------------------
data4  = [Vy;Vx;rotation;w;steering;w_out];

trnData=data4';
numMFs=2;
mfType='gbellmf';
in_fis = genfis1(trnData,numMFs,mfType);

[out_fis4,~,stepSize4] = anfis(trnData,in_fis,300);

figure;
plot(stepSize4);
grid on
xlabel('Iteration times');
ylabel('Anfis training error');
title('Sub-system ANGULAR VELOCITY case');

figure;
plotfis(in_fis);
title('Input-Output scheme for the sub-system ANGULAR VELOCITY case');

figure;
plot(test_data_w(6,:),'b');
hold on
plot(evalfis(test_data_w([1 2 3 4 5],:)',out_fis4),'r');
ylabel('Angular velocity');
xlabel('Time step');
legend('Vehicle output','ANFIS prediction output');
title('Sub-system ANGULAR VELOCITY case');
%--------------------------------------------------------------------------

save result1.mat out_fis1 out_fis2 out_fis3 out_fis4

