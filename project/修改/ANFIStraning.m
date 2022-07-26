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
xlabel('迭代次数');
ylabel('anfis training error');
title('sub-system 1 case');

figure;
plotfis(in_fis);
title('Input-Output scheme for the sub-system 1 case');

figure;
plot(test_data_Vy(6,:),'b');
hold on
plot(evalfis(test_data_Vy([1 2 3 4 5],:)',out_fis1),'r');
legend('Vehicle output','ANFIS output');
title('sub-system 1 case');



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
xlabel('迭代次数');
ylabel('anfis training error');
title('sub-system 2 case');

figure;
plotfis(in_fis);
title('Input-Output scheme for the sub-system 2 case');

figure;
plot(test_data_Vx(6,:),'b');
hold on
plot(evalfis(test_data_Vx([1 2 3 4 5],:)',out_fis2),'r');
legend('Vehicle output','ANFIS output');
title('sub-system 2 case');



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
xlabel('迭代次数');
ylabel('anfis training error');
title('sub-system 3 case');

figure;
plotfis(in_fis);
title('Input-Output scheme for the sub-system 3 case');

figure;
plot(test_data_rotation(6,:),'b');
hold on
plot(evalfis(test_data_rotation([1 2 3 4 5],:)',out_fis3),'r');
legend('Vehicle output','ANFIS output');
title('sub-system 3 case');


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
xlabel('迭代次数');
ylabel('anfis training error');
title('sub-system 4 case');

figure;
plotfis(in_fis);
title('Input-Output scheme for the sub-system 4 case');

figure;
plot(test_data_w(6,:),'b');
hold on
plot(evalfis(test_data_w([1 2 3 4 5],:)',out_fis4),'r');
legend('Vehicle output','ANFIS output');
title('sub-system 4 case');
%--------------------------------------------------------------------------

save result1.mat out_fis1 out_fis2 out_fis3 out_fis4

