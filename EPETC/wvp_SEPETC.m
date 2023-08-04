function [sys,x0,str,ts,simStateCompliance] = wvp_SEPETC(t,x,u,flag)
switch flag,
    case 0,
        [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes;
    case 1,
        sys=mdlDerivatives(t,x,u);
    case 2,
        sys=mdlUpdate(t,x,u);
    case 3,      
        sys=mdlOutputs(t,x,u); 
    case 4,
        sys=mdlGetTimeOfNextVarHit(t,x,u);
    case 9,
        sys=mdlTerminate(t,x,u);
    otherwise
        DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));        
end

function [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes()
sizes = simsizes;
sizes.NumContStates  = 0;
sizes.NumDiscStates  = 1;
sizes.NumOutputs     = 32; 
%% SYS=[x1_hat,x1_o2_hat,x1_o3_hat,x1_04_hat,...,flag1....z_kh,zw_1,...，zv_1,...]';
sizes.NumInputs      = 28; 
%% u(1)=x1_o1;u(2)=x1_o2;u(3)=x1_o3;u4=x1_04,...u(17)=x1...,u(21)=w1,...,u(25)=v1,...;
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 1;   % at least one sample time is needed
sys = simsizes(sizes);
%% 可更改
%% 事件驱动条件 delta=0.5;
global ci h ci_v hv
ci=[0.03;0.03;0.03;0.03];
h=0.06; % cp=1;
ci_v=[0.004;0.004;0.004;0.004];
hv=0.02; % cv=5;  
%% 仿真时间设置
% t1=0;               % 0s 连接
% t2=1;               % 1s 二次控制发挥作用
% t3=10;              % 10s load 2 on
% t4=20;              % 25s load 2 off
% t5=35;              % 45s DG 3 off
% t6=40;              % 50s DG 3 on
% % Tend=50;
global t2 t4 t5 t6 t7 Tend 
t2=1; % 二次控制开始时间
t4=21; % link failure 1-4
t5=33; % link restoration of 1-4
t6=35; % DG 3 off
t7=40; % DG 3 on
Tend=10;


%% 控制输出
global zw_kh zv_kh z_kh
zw_kh=zeros(4,1);zv_kh=zeros(4,1);z_kh=zeros(4,1);

%% 拓扑矩阵
global G L1 L2 L3
A1=[0 1 0 1;1 0 1 0;0 1 0 1;1 0 1 0];
D1=diag([2,2,2,2]);
G=diag([1,0,0,0]);
L1=D1-A1;
%% DG 3 OFF
A2=[0 1 0 1;1 0 0 0;0 0 0 0;1 0 0 0];
D2=diag([2,1,0,1]);
L2=D2-A2;
%% Link Failure 1-4
A3=[0 1 0 0;1 0 1 0;0 1 0 1;0 0 1 0];
D3=diag([1,2,2,1]);
L3=D3-A3;
%% 标称值
global w_ref v_ref
w_ref=2*pi*60;v_ref=380;
%% 事件驱动时刻值
global x_hat w_hat v_hat
x_hat=zeros(4,1);w_hat=zeros(4,1);v_hat=zeros(4,1);
%% 事件驱动标志
global ETC_flag ETC_flagv
ETC_flag=zeros(4,1); ETC_flagv=zeros(4,1);
%% 事件驱动时刻对邻居的估计
global x1_on_hat x2_on_hat x3_on_hat x4_on_hat
x1_on_hat=zeros(3,1);x2_on_hat=zeros(3,1);x3_on_hat=zeros(3,1);x4_on_hat=zeros(3,1);
%% 驱动序列,记录事件驱动时刻（t,x1_hat）
global tic tp tv
global TP TW TV
TP=cell(4,1);TW=cell(4,1);TV=cell(4,1);
tic=ones(4,1);tp=ones(4,1);tv=ones(4,1);

%% 初始化x %%不含求导变量
x0  = 0;
str = [];
ts  = [0.02 0];
simStateCompliance = 'UnknownSimState';

function sys=mdlDerivatives(t,x,u)

dx=0;
sys = dx;

function sys=mdlUpdate(t,x,u)
global h hv t2 t4 t5 t6 t7 L1 L2 L3
if t>=t2 % 1秒前为一次控制
    %% 拓扑
% if t>t4 && t<t5 %link 1-4 failure 
%     L=L3;
% else
%     if t>t6 && t<t7 % DG 3 OFF
%        L=L2;
%     else
%        L=L1;
%     end
% end
L=L1;

    %% 周期性判断驱动条件
     if mod(t,h)==0 
       etc_dp(u,t,L);
    end    
    %%  vlotage 
    if mod(t,hv)==0   
       etc_dv(u,t,L);    
    end
end

sys = [];

function sys=mdlOutputs(t,x,u)

global z_kh zw_kh zv_kh
global x_hat ETC_flag 
global x1_on_hat x2_on_hat x3_on_hat x4_on_hat
global TW TP TV Tend

if t==Tend
    save TP 
    save TW 
    save TV
end

sys=[x_hat(1,1),x_hat(2,1),x_hat(3,1),x_hat(4,1),...
    ETC_flag(1,1),ETC_flag(2,1),ETC_flag(3,1),ETC_flag(4,1),...
    z_kh(1,1),z_kh(2,1),z_kh(3,1),z_kh(4,1),...
    zv_kh(1,1),zv_kh(2,1),zv_kh(3,1),zv_kh(4,1),...
    x2_on_hat(1,1),x2_on_hat(2,1),x2_on_hat(3,1),...
    x3_on_hat(1,1),x3_on_hat(2,1),x3_on_hat(3,1),...
    x4_on_hat(1,1),x4_on_hat(2,1),x4_on_hat(3,1),...
    zw_kh(1,1),zw_kh(2,1),zw_kh(3,1),zw_kh(4,1),...
    x1_on_hat(1,1),x1_on_hat(2,1),x1_on_hat(3,1)]';

function sys=mdlGetTimeOfNextVarHit(t,x,u)

sampleTime = 1;    %  Example, set the next hit to be one second later.
sys = t + sampleTime;

function sys=mdlTerminate(t,x,u)

sys = [];


function etc_dp(u,prac_t,L)
%% 事件驱动函数
global x1_on_hat x2_on_hat x3_on_hat x4_on_hat
global G 
global t4 t5 t6 t7
global ci w_ref ETC_flag
global x_hat w_hat zw_kh z_kh 
global TP TW tic tp 
%     input pi=[u(1) u(2) u(3) u(4);
%         u(5) u(6) u(7) u(8);
%         u(9) u(10) u(11) u(12);
%         u(13) u(14) u(15) u(16)];
% input wi=[u(21) u(22) u(23) u(24)]'
%%
    z_kh(1,1)=-L(1,:)*[u(1),u(2),u(3),u(4)]';
    z_kh(2,1)=-L(2,:)*[u(5) u(6) u(7) u(8)]';
    z_kh(3,1)=-L(3,:)*[u(9) u(10) u(11) u(12)]';
    z_kh(4,1)=-L(4,:)*[u(13) u(14) u(15) u(16)]';

    pi_oi=[u(1) u(6) u(11) u(16)]';
    pi=[u(17) u(18) u(19) u(20)]';
    flagg=(pi-pi_oi).*(pi-pi_oi)-diag(ci)*(z_kh.*z_kh);

    % 更新驱动标志 flag,xi_hat,xi_oni,w_hat 
        
    for i=1:4       
        if i==3 && (prac_t<t7 && prac_t>t6)            
            flagg(i)=-1;            
        end
        if flagg(i)>0
            ETC_flag(i,1)=1;
            x_hat(i,1)=u(16+i); w_hat(i,1)=u(20+i); % 更新
            TP{i,1}(tp(i)+1,1:2)=[prac_t,x_hat(i,1)];
            TW{i,1}(tic(i)+1,1:2)=[prac_t,w_hat(i,1)];
            tp(i)=tp(i)+1;tic(i)=tic(i)+1;
        else 
            ETC_flag(i,1)=0;
        end
    end 
    
    if flagg(1,1)>0
        x1_on_hat(1:3,1)=[u(2);u(3);u(4)];
  %      x1_on_hat(1,1)=u(2);x1_on_hat(2,1)=u(3);x1_on_hat(3,1)=u(4); % 更新事件驱动时刻对邻居的估计
        z_kh(1,1)=-L(1,:)*[x_hat(1,1) u(2) u(3) u(4)]'; % 更新pi控制作用   
    end
    if flagg(2,1)>0
        x2_on_hat(1:3,1)=[u(5);u(7);u(8)];
   %     x2_on_hat(1,1)=u(5);x2_on_hat(2,1)=u(7);x2_on_hat(3,1)=u(8);
        z_kh(2,1)=-L(2,:)*[u(5) x_hat(2,1) u(7) u(8)]';
    end
    if flagg(3,1)>0 
        x3_on_hat(1:3,1)=[u(9);u(10);u(12)];
     %   x3_on_hat(1,1)=u(9);x3_on_hat(2,1)=u(10);x3_on_hat(3,1)=u(12);
        z_kh(3,1)=-L(3,:)*[u(9) u(10) x_hat(3,1) u(12)]';
    end

    if flagg(4,1)>0 
        x4_on_hat(1:3,1)=[u(13);u(14);u(15)];
  %      x4_on_hat(1,1)=u(13);x4_on_hat(2,1)=u(14);x4_on_hat(3,1)=u(15);
        z_kh(4,1)=-L(4,:)*[u(13) u(14) u(15) x_hat(4,1)]';
    end  
    
zw_kh=-(L+G)*(w_hat-w_ref*ones(4,1)); 

function etc_dv(u,prac_t,L)

global ci_v  v_ref v_hat zv_kh
global G t2 t4 t5 t6 t7  
global ETC_flagv TV tv

%%
vi_hat=v_hat;
vi=[u(25) u(26) u(27) u(28)]';
zv_kh=-(L+G)*(v_hat-v_ref*ones(4,1));
flagg=(vi-vi_hat).*(vi-vi_hat)-diag(ci_v)*(zv_kh.*zv_kh);

%  更新驱动标志
if prac_t==t2
    ETC_flagv=[1;1;1;1];
    v_hat=[u(25) u(26) u(27) u(28)]';
else
    for i=1:4
         if i==1
            if abs(u(25)-v_ref)<2e-3
                flagg(i)=-1;
            end
         else
            if i==3
                if abs(zv_kh(i,1))<2e-3 || (prac_t<t7 && prac_t>t6)
                    flagg(i)=-1;
                end
            else
                if abs(zv_kh(i,1))<2e-3
                    flagg(i)=-1;
                end              
            end
         end
         if flagg(i)>0
             ETC_flagv(i,1)=1;
             v_hat(i,1)=u(24+i);
             TV{i,1}(tv(i)+1,1:2)=[prac_t,v_hat(i,1)];
             tv(i)=tv(i)+1;
         else
             ETC_flagv(i,1)=0;
         end
    end  
end
zv_kh=-(L+G)*(v_hat-v_ref*ones(4,1));










