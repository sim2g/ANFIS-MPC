clc;
clear;



load result1.mat
%current time sterp vehicle state from unity = [Vy, Vx, rotation, angular velocity,steering]
x = [0.038745,14.85566,-1.61183,0.37818,0.35498];

tic
[output1,~,~,~,ruleFiring1] = evalfis(x,out_fis1);
[output2,~,~,~,ruleFiring2] = evalfis(x,out_fis2);
[output3,~,~,~,ruleFiring3] = evalfis(x,out_fis3);
[output4,~,~,~,ruleFiring4] = evalfis(x,out_fis4);

toc


tic
structure = zeros(4,6);
outfis = [out_fis1,out_fis2,out_fis3,out_fis4];
ruleFiring = [ruleFiring1,ruleFiring2,ruleFiring3,ruleFiring4];


for o = 1:length(outfis)
    array = zeros(32,6);
    rule = ruleFiring(1:32,o);
    S = sum(rule,'all');
    for i = 1:32
        c = outfis(o).output.mf;
        consequent_para = c(i).params;
        layer_four = rule(i)*consequent_para/S;
        for j = 1:6
            array(i,j) = layer_four(j);
        end
    end
    for k = 1:6
        summation = sum(array,1);
        structure(o,k) = summation(k);
    end
end


predict_values = zeros(1,32);
for i = 1:32
    out = array(i,1)*x(1) + array(i,2)*x(2) + array(i,3)*x(3) + array(i,4)*x(4) + array(i,5)*x(5) + array(i,6);
    predict_values(1,i) = out;
end

Final = sum(predict_values,'all');

final = structure(4,1)*x(1) + structure(4,2)*x(2) + structure(4,3)*x(3) + structure(4,4)*x(4) + structure(4,5)*x(5) + structure(4,6);

toc
