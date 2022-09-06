function [sys,x0,str,ts] = MPCC(t,x,u,flag)

% State veriable=[y_dot,x_dot,phi,phi_dot,Y,X]£¬control variable=delta_f


switch flag,
 case 0
  [sys,x0,str,ts] = mdlInitializeSizes; % Initialization
  
 case 2
  sys = mdlUpdates(t,x,u); % Update discrete states
  
 case 3
  sys = mdlOutputs(t,x,u); % Calculate outputs
 
%  case 4
%   sys = mdlGetTimeOfNextVarHit(t,x,u); % Get next sample time 

 case {1,4,9} % Unused flags
  sys = [];
  
 otherwise
  error(['unhandled flag = ',num2str(flag)]); % Error handling
end
% End of dsfunc.

%==============================================================
% Initialization
%==============================================================

function [sys,x0,str,ts] = mdlInitializeSizes

% Call simsizes for a sizes structure, fill it in, and convert it 
% to a sizes array.

sizes = simsizes;
sizes.NumContStates  = 0;
sizes.NumDiscStates  = 6;
sizes.NumOutputs     = 1;
sizes.NumInputs      = 6;
sizes.DirFeedthrough = 1; % Matrix D is non-empty.
sizes.NumSampleTimes = 1;
sys = simsizes(sizes); 
x0 =[0.001;0.0001;0.0001;0.00001;0.00001;0.00001];    
global U;
U=[0];%Initialisation of the control out
% global x;
% x = zeros(md.ne + md.pye + md.me + md.Hu*md.me,1);   
% Initialize the discrete states.
str = [];             % Set str to an empty matrix.
ts  = [0.04 0];       % sample time: [period, offset]
%End of mdlInitializeSizes
		      
%==============================================================
% Update the discrete states
%==============================================================
function sys = mdlUpdates(t,x,u)
  
sys = x;
%End of mdlUpdate.

%==============================================================
% Calculate outputs
%==============================================================
function sys = mdlOutputs(t,x,u)
    global a b; 
    %global u_piao;
    global U;
    %global kesi;
    tic%Start timing
    Nx=6;%Number of state variable
    Nu=1;%Number of control varibale
    Ny=2;%Output number
    Np =20;%Prediction steps
    Nc=10;%Control steps
    Row=1000;%Relxation factor, ensure there is a solution
    fprintf('Update start, t=%6.3f\n',t)
   
    phi=2*asin(u(3)); %rotation In radian, fixing the data we read from Unity 3d
    y_dot=u(1)*cos(phi)-u(2)*sin(phi);%We want the velocity in the body frame
    x_dot=u(2)*cos(phi)+u(1)*sin(phi);%m/s
    
    phi_dot=u(4);%Angular velocity
    Y=u(5);%m
    X=u(6);%m
    
%% Vehicle parameter setup
%syms sf sr are the front and back wheel slip ratio, these value can be
%estimated or found online
    Sf=0.2; Sr=0.2;
%syms lf lr;%Distance between front and back wheel to the centre of mass,
%from unity
    lf=1.232;lr=1.468;
%syms C_cf C_cr C_lf C_lr;%Front and back wheel lateral and longitudinal
%slip coefficient
    Ccf=66900;Ccr=62700;Clf=66900;Clr=62700;
%syms m g I;%m=mass£¬g=Accelaration due to gravity£¬I=inertia around z axis
    m=1600;g=9.8;I=4175;
   

%% Reference track parameters setup
    shape=2.4;
    dx1=25;dx2=21.95;
    dy1=54.05;dy2=5.7;
    Xs1=27.19;Xs2=56.46;


    X_predict=zeros(Np,1);%Store the x position of the vehicle in the prediction horizon
    phi_ref=zeros(Np,1);%Store the rotation of the reference track in the prediction horizon
    Y_ref=zeros(Np,1);%Store the y positon of the reference track
    
    %  Calcualte kesi, conbine the system state and control variable  
    kesi=zeros(Nx+Nu,1);
    kesi(1)=y_dot;%u(1)==X(1)
    kesi(2)=x_dot;%u(2)==X(2)
    kesi(3)=phi; %u(3)==X(3)
    kesi(4)=phi_dot;
    kesi(5)=Y;
    kesi(6)=X;
    kesi(7)=U(1);
    delta_f=U(1);
    fprintf('Update start, u(1)=%4.2f\n',U(1))

    T=0.04;%simulation step
    T_all=60;%Total simulation time
     
    %Weight matrix setup for Q and R i nthe cost function
    Q_cell=cell(Np,Np);
    for i=1:1:Np;
        for j=1:1:Np;
            if i==j
                %Q_cell{i,j}=[200 0;0 100;];
                Q_cell{i,j}=[2000 0;0 10000];
            else 
                Q_cell{i,j}=zeros(Ny,Ny);               
            end
        end 
    end 
    R=5*10^4*eye(Nu*Nc);
    %R=5*10^5*eye(Nu*Nc);
    %Matrix after Jocabine calculation, see and run document dynamic model
    a=[                 1 - (259200*T)/(1723*x_dot),                                                         -T*(phi_dot + (2*((460218*phi_dot)/5 - 62700*y_dot))/(1723*x_dot^2) - (133800*((154*phi_dot)/125 + y_dot))/(1723*x_dot^2)),                                    0,                     -T*(x_dot - 96228/(8615*x_dot)), 0, 0
        T*(phi_dot - (133800*delta_f)/(1723*x_dot)),                                                                                                                  (133800*T*delta_f*((154*phi_dot)/125 + y_dot))/(1723*x_dot^2) + 1,                                    0,           T*(y_dot - (824208*delta_f)/(8615*x_dot)), 0, 0
                                                  0,                                                                                                                                                                                  0,                                    1,                                                   T, 0, 0
            (33063689036759*T)/(7172595384320*x_dot), T*(((2321344006605451863*phi_dot)/8589934592000 - (6325188028897689*y_dot)/34359738368)/(4175*x_dot^2) + (5663914248162509*((154*phi_dot)/125 + y_dot))/(143451907686400*x_dot^2)),                                   0, 1 - (813165919007900927*T)/(7172595384320000*x_dot), 0, 0
                                          T*cos(phi),                                                                                                                                                                         T*sin(phi),  T*(x_dot*cos(phi) - y_dot*sin(phi)),                                                   0, 1, 0
                                         -T*sin(phi),                                                                                                                                                                         T*cos(phi), -T*(y_dot*cos(phi) + x_dot*sin(phi)),                                                   0, 0, 1];
    
    b=[                                                               133800*T/1723
       T*((267600*delta_f)/1723 - (133800*((154*phi_dot)/125 + y_dot))/(1723*x_dot))
                                                                                 0
                                                5663914248162509*T/143451907686400
                                                                                 0
                                                                                 0];  
    d_k=zeros(Nx,1);%for calculate error after linearisatione
    state_k1=zeros(Nx,1);%predict the next time step state value
    %According to the descrete nonlinear model,SEE REPORT
    state_k1(1,1)=y_dot+T*(-x_dot*phi_dot+2*(Ccf*(delta_f-(y_dot+lf*phi_dot)/x_dot)+Ccr*(lr*phi_dot-y_dot)/x_dot)/m);
    state_k1(2,1)=x_dot+T*(y_dot*phi_dot+2*(Clf*Sf+Clr*Sr+Ccf*delta_f*(delta_f-(y_dot+phi_dot*lf)/x_dot))/m);
    state_k1(3,1)=phi+T*phi_dot;
    state_k1(4,1)=phi_dot+T*((2*lf*Ccf*(delta_f-(y_dot+lf*phi_dot)/x_dot)-2*lr*Ccr*(lr*phi_dot-y_dot)/x_dot)/I);
    state_k1(5,1)=Y+T*(x_dot*sin(phi)+y_dot*cos(phi));
    state_k1(6,1)=X+T*(x_dot*cos(phi)-y_dot*sin(phi));
    d_k=state_k1-a*kesi(1:6,1)-b*kesi(7,1);%find the error for each state
    d_piao_k=zeros(Nx+Nu,1);%Error expression matrix
    d_piao_k(1:6,1)=d_k;
    d_piao_k(7,1)=0;
    
    %Setup new A/B matrix
    A_cell=cell(2,2);
    B_cell=cell(2,1);
    A_cell{1,1}=a;
    A_cell{1,2}=b;
    A_cell{2,1}=zeros(Nu,Nx);
    A_cell{2,2}=eye(Nu);
    B_cell{1,1}=b;
    B_cell{2,1}=eye(Nu);
    %A=zeros(Nu+Nx,Nu+Nx);
    A=cell2mat(A_cell);
    B=cell2mat(B_cell);
   % C=[0 0 1 0 0 0 0;0 0 0 1 0 0 0;0 0 0 0 1 0 0;];%output matrix
    C=[0 0 1 0 0 0 0;0 0 0 0 1 0 0;];
    %Construct the new state space representation with the following
    %matrixs setup
    PSI_cell=cell(Np,1);
    THETA_cell=cell(Np,Nc);
    GAMMA_cell=cell(Np,Np);
    PHI_cell=cell(Np,1);
    for p=1:1:Np;
        PHI_cell{p,1}=d_piao_k;
        for q=1:1:Np;
            if q<=p;
                GAMMA_cell{p,q}=C*A^(p-q);
            else 
                GAMMA_cell{p,q}=zeros(Ny,Nx+Nu);
            end 
        end
    end
    for j=1:1:Np
     PSI_cell{j,1}=C*A^j;
        for k=1:1:Nc
            if k<=j
                THETA_cell{j,k}=C*A^(j-k)*B;
            else 
                THETA_cell{j,k}=zeros(Ny,Nu);
            end
        end
    end
    PSI=cell2mat(PSI_cell);%size(PSI)=[Ny*Np Nx+Nu]
    THETA=cell2mat(THETA_cell);%size(THETA)=[Ny*Np Nu*Nc]
    GAMMA=cell2mat(GAMMA_cell);%´óÐ´µÄGAMMA
    PHI=cell2mat(PHI_cell);
    Q=cell2mat(Q_cell);
    %quatrotic programming transformation
    H_cell=cell(2,2);
    H_cell{1,1}=THETA'*Q*THETA+R;
    H_cell{1,2}=zeros(Nu*Nc,1);
    H_cell{2,1}=zeros(1,Nu*Nc);
    H_cell{2,2}=Row;
    H=cell2mat(H_cell);
    H=(H+H')/2;
    error_1=zeros(Ny*Np,1);
    %Setup the reference state, we have four road scenarios, find the
    %position Y and rotation according to x at each time step
    Yita_ref_cell=cell(Np,1);
    for p=1:1:Np
        if t+p*T>T_all
            X_DOT=x_dot*cos(phi)-y_dot*sin(phi);%longitudinal velocity in catesian coordinate
            X_predict(Np,1)=X+X_DOT*Np*T;
            %X_predict(Np,1)=X+X_dot*Np*t;
            z1=shape/dx1*(X_predict(Np,1)-Xs1)-shape/2;
            z2=shape/dx2*(X_predict(Np,1)-Xs2)-shape/2;
            %Y_ref(p,1)=dy1/2*(1+tanh(z1))-dy2/2*(1+tanh(z2));
            %phi_ref(p,1)=atan(dy1*(1/cosh(z1))^2*(1.2/dx1)-dy2*(1/cosh(z2))^2*(1.2/dx2));

            %Test case 2
            %Y_ref(p,1)=0.005*(X_predict(Np,1))^2;
            %phi_ref(p,1)=atan(0.01*X_predict(Np,1));

            %Test case 4
            Y_ref(p,1)=(6859*X_predict(Np,1)^3*(20577*X_predict(Np,1)^2-3781760*X_predict(Np,1)+175110000))/1000000000000000;
            %phi_ref(p,1)=atan((6859*X_predict(Np,1)^2*(20577*X_predict(Np,1)^2-3025408*X_predict(Np,1)+105066000))/200000000000000);             
            
            %Test case 1
            %Y_ref(p,1)=X_predict(Np,1);
            %phi_ref(p,1)=atan(1);  


            
            Yita_ref_cell{p,1}=[phi_ref(p,1);Y_ref(p,1)];
            
        else
            X_DOT =x_dot*cos(phi)-y_dot*sin(phi);%longitudinal velocity in catesian coordinate
            X_predict(p,1)=X+X_DOT*p*T;%Find X position first£¬X(t)=X+X_dot*t
            z1=shape/dx1*(X_predict(p,1)-Xs1)-shape/2;
            z2=shape/dx2*(X_predict(p,1)-Xs2)-shape/2;
            %Y_ref(p,1)=dy1/2*(1+tanh(z1))-dy2/2*(1+tanh(z2));
            %phi_ref(p,1)=atan(dy1*(1/cosh(z1))^2*(1.2/dx1)-dy2*(1/cosh(z2))^2*(1.2/dx2));
            
            %Test case 2
            %Y_ref(p,1)=0.005*(X_predict(p,1))^2;
            %phi_ref(p,1)=atan(0.01*X_predict(p,1));

            %Test case 4
            Y_ref(p,1)=(6859*X_predict(p,1)^3*(20577*X_predict(p,1)^2-3781760*X_predict(p,1)+175110000))/1000000000000000;
            phi_ref(p,1)=atan((6859*X_predict(p,1)^2*(20577*X_predict(p,1)^2-3025408*X_predict(p,1)+105066000))/200000000000000);            

            %Test case 1
            %Y_ref(p,1)=X_predict(p,1);
            %phi_ref(p,1)=atan(1);    



            Yita_ref_cell{p,1}=[phi_ref(p,1);Y_ref(p,1)];

        end
    end
    Yita_ref=cell2mat(Yita_ref_cell);
    error_1=Yita_ref-PSI*kesi-GAMMA*PHI; %Find error
    f_cell=cell(1,2);
    f_cell{1,1}=2*error_1'*Q*THETA; %see matlab quatrotic programming rules
    f_cell{1,2}=0;
%     f=(cell2mat(f_cell))';
    f=-cell2mat(f_cell);
    
 %% Setup constriants
 %Control constraints
    A_t=zeros(Nc,Nc);%falcone report P181
    for p=1:1:Nc
        for q=1:1:Nc
            if q<=p 
                A_t(p,q)=1;
            else 
                A_t(p,q)=0;
            end
        end 
    end 
    A_I=kron(A_t,eye(Nu));%Find Kronecker
    Ut=kron(ones(Nc,1),U(1));
    umin=-0.6108;%steering constraints
    umax=0.6108;
    delta_umin=-0.05;%rate of change of steering
    delta_umax=0.05;
    Umin=kron(ones(Nc,1),umin);
    Umax=kron(ones(Nc,1),umax);
    
    %Output constriants
    ycmax=[1.6;1000]; %Position Y
    ycmin=[-1.6;-100];
    Ycmax=kron(ones(Np,1),ycmax);
    Ycmin=kron(ones(Np,1),ycmin);
    
    %Combine the constriants, see report constraints
    A_cons_cell={A_I zeros(Nu*Nc,1);-A_I zeros(Nu*Nc,1);THETA zeros(Ny*Np,1);-THETA zeros(Ny*Np,1)};
    b_cons_cell={Umax-Ut;-Umin+Ut;Ycmax-PSI*kesi-GAMMA*PHI;-Ycmin+PSI*kesi+GAMMA*PHI};
    A_cons=cell2mat(A_cons_cell);%(Solving equation) State quantity inequality constraint gain matrix, converted to the range of absolute values
    b_cons=cell2mat(b_cons_cell);%(Solving the equation) The value of the state quantity inequality constraint
    
    %State constriants
    M=10; 
    delta_Umin=kron(ones(Nc,1),delta_umin);
    delta_Umax=kron(ones(Nc,1),delta_umax);
    lb=[delta_Umin;0];%(solving equation) lower bound of state quantity, including control increment and relaxation factor in control time domain
    ub=[delta_Umax;M];%(solving equation) upper bound of state quantity, including control increment and relaxation factor in control time domain
    
    %% Solve quadprog for cost function
       options = optimset('Algorithm','active-set');
       x_start=zeros(Nc+1,1);%Add a starting point
      [X,fval,exitflag]=quadprog(H,f,A_cons,b_cons,[],[],lb,ub,x_start,options);
      fprintf('exitflag=%d\n',exitflag);
      fprintf('H=%4.2f\n',H(1,1));
      fprintf('f=%4.2f\n',f(1,1));
    %% Calculate output
    u_piao=X(1);%increment of steering angle
    U(1)=kesi(7,1)+u_piao;%current control action= last time step control+increment
   %U(2)=Yita_ref(2);%Output dphi_ref
    sys= U;
    toc
% End of mdlOutputs.


