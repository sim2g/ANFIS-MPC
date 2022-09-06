%% Jocabine matrix calculation
clc
clear all;

%Vihicle parameters
syms x_dot y_dot phi phi_dot Y X;%Vehicle state 
syms delta_f  %Front wheel rotate angle
%syms sf sr are the front and back wheel slip ratio, these value can be
%estimated or found online
Sf=0.2; Sr=0.2;
%syms a b;%Distance between front and back wheel to the centre of mass
a=1.232;b=1.468;
%syms C_cf C_cr C_lf C_lr;%Front and back wheel lateral and longitudinal
%slip coefficient
Ccf=66900;Ccr=62700;Clf=66900;Clr=62700;
%syms m g I;%m=mass£¬g=Accelaration due to gravity£¬I=inertia around z axis
m=1723;g=9.8;I=4175;

% Vehicle dynamic model, representation of the vehicle state from force
% analysis
dy_dot=-x_dot*phi_dot+2*(Ccf*(delta_f-(y_dot+a*phi_dot)/x_dot)+Ccr*(b*phi_dot-y_dot)/x_dot)/m;
dx_dot=y_dot*phi_dot+2*(Clf*Sf+Clr*Sr+Ccf*delta_f*(delta_f-(y_dot+phi_dot*a)/x_dot))/m;
dphi_dot=(2*a*Ccf*(delta_f-(y_dot+a*phi_dot)/x_dot)-2*b*Ccr*(b*phi_dot-y_dot)/x_dot)/I;
Y_dot=x_dot*sin(phi)+y_dot*cos(phi);
X_dot=x_dot*cos(phi)-y_dot*sin(phi);

% Solve the Jocabine matrix to find the linear representation of the
% vehicle state we want
f=[dy_dot;dx_dot;phi_dot;dphi_dot;Y_dot;X_dot];%vehicle dynamic
kesi=[y_dot,x_dot,phi,phi_dot,Y,X];%our system state we want the expression
v=delta_f;%control state
R=jacobian(f,kesi);%Matrix A(t)-continous
R2=jacobian(f,v);%MatrixB(t)-continous

% transfer the continous matrix to discrete matrix  A=I+T*A(t),B=T*B(t))
I=eye(6);
syms T;
A=I+T*R;
B=T*R2;
A1=vpa(A,3);
B1=vpa(B,3);
%Obtain the final linear representation of the vehicle dynamic 

