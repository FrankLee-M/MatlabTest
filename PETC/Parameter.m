
%% hv=0.02;
%% Fundamental Para
fref = 60;          %Hz
wref = 2*pi*fref;   %rad/s
fb = 60;            %Hz
wb = 2*pi*fb;       %rad/s
vref = 380;         %V
wc = 31.4;        %rad/s
T = 5e-5;           %s


%% S函数需做出对应更改 Tend
t1=0;               % 0s 连接
t2=1;               % 1s 二次控制发挥作用
t3=10;              % 10s load 2 on
t4=20;              % 25s load 2 off
t5=35;              % 45s DG 3 off
t6=40;              % 50s DG 3 on
% Tend=50;
%%
Uo=220*sqrt(2);

% % voltage PI
% kivp=5;
% kivi=10;
% control gain
cw = 5;
cp = 1;
cv = 5;
cq = 1;


% DG droop parameter
Mp1 = 9.4e-5;
Nq1 = 1.3e-3;

Mp2 = 9.4e-5;
Nq2 = 1.3e-3;

Mp3 = 12.5e-5;
Nq3 = 1.5e-3;

Mp4 = 12.5e-5;
Nq4 = 1.5e-3;



% heavy load 

PL1 = 45.9e3;         % W
QL1 = 22.8e3;         % Var
PL2 = 45.9e3;         % W
QL2 = 22.8e3;         % Var

PL3 = 36e3;         % W
QL3 = 36e3;          % Var

PL4 = 36e3;         % W
QL4 = 36e3;          % Var

%Output Connector
Rc1 = 0.03;         % Ohm
Lc1 = 0.35e-3;      % H
Rc2 = 0.03;         % Ohm
Lc2 = 0.35e-3;      % H
Rc3 = 0.03;         % Ohm
Lc3 = 0.35e-3;      % H
Rc4 = 0.03;         % Ohm
Lc4 = 0.35e-3;      % H



% % %VOL PID
% % Kpv1 = 0.1;
% % Kiv1 = 420;
% % Kdv1 = 0;
% % Kpc1 = 10;
% % Kic1 = 20;
% % Kdc1 = 0;
% % 
% % Kpv2 = Kpv1;
% % Kiv2 = Kiv1;
% % Kdv2 = Kdv1;
% % Kpc2 = Kpc1;
% % Kic2 = Kic1;
% % Kdc2 = Kdc1;
% % 
% % Kpv3 = Kpv1;
% % Kiv3 = Kiv1;
% % Kdv3 = Kdv2;
% % Kpc3 = Kpc1;
% % Kic3 = Kic1;
% % Kdc3 = Kdc1;
% % 
% % Kpv4 = Kpv3;
% % Kiv4 = Kiv3;
% % Kdv4 = Kdv3;
% % Kpc4 = Kpc3;
% % Kic4 = Kic3;
% % Kdc4 = Kdc3;
% % 
% % % VOL
% % VOL1 = 500;         %V
% % VOL2 = 500;         %V
% % VOL3 = 500;         %V
% % VOL4 = 500;         %V

%transLine
Rt12 = 0.23;         %Ω
Lt12 = 318e-6;       % H
Rt23 = 0.35;         %Ω
Lt23 = 1847e-6;       % H
Rt34 = 0.23;         %Ω
Lt34 = 318e-6;       % H

%Filter
Rf1 = 0.1;          % Ohm
Lf1 = 1.35e-3;      % H
Cf1 = 50e-6;        % F

Rf2 = 0.1;          % Ohm
Lf2 = 1.35e-3;      % H
Cf2 = 50e-6;        % F

Rf3 = 0.1;          % Ohm
Lf3 = 1.35e-3;      % H
Cf3 = 50e-6;        % F

Rf4 = 0.1;          % Ohm
Lf4 = 1.35e-3;      % H
Cf4 = 50e-6;        % F


