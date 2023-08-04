function [sys,x0,str,ts,simStateCompliance] = wvpq_PETC(t,x,u,flag)
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
sizes.NumOutputs     = 12;
% SYS=[zw1,...,zv1,...,zp1,...,zq1,...]';
sizes.NumInputs      = 12;
% u=[w1,...,v1,...,p1,...,q1,...]
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 1;   % at least one sample time is needed
sys = simsizes(sizes);
%% 可更改：
% t1=0;               % 0s 连接
% t2=1;               % 1s 二次控制发挥作用
% t3=10;              % 10s load 2 on
% t4=20;              % 25s load 2 off
% t5=35;              % 45s DG 3 off
% t6=45;              % 50s DG 3 on
% % Tend=50;
global t2 t4 t5 t6 t7 Tend
t2=1; % 二次控制开始时间
t4=21; % link failure 1-4
t5=33; % link restoration of 1-4
t6=35; % DG 3 off
t7=40; % DG 3 on
Tend=10;



%% 事件驱动条件 
global ci_p ci_v
%% δ=0.5；
ci_p=[0.03;0.03;0.03;0.03];
%% delta = 0.1
ci_v=[0.004;0.004;0.004;0.004]; 

global hp hv
hp=0.06; % cp=1;
hv=0.02; % cv=5;

%%  拓扑矩阵
global G L1 L2 L3
A1=[0 1 0 1;1 0 1 0;0 1 0 1;1 0 1 0];
D1=diag([2,2,2,2]);
G=diag([1,0,0,0]);
L1=D1-A1;
%% DG OFF
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
%% 控制输出
global zw_kh zv_kh zp_kh
zw_kh=zeros(4,1);zv_kh=zeros(4,1);zp_kh=zeros(4,1);

%% 事件驱动时刻值
global w_hat v_hat p_hat
w_hat=zeros(4,1);v_hat=zeros(4,1);p_hat=zeros(4,1);
%% 事件驱动标志
global ETC_flagp ETC_flagv
ETC_flagp=zeros(4,1); ETC_flagv=zeros(4,1);

%% 驱动序列,记录事件驱动时刻（t,x1_hat）
global tic tp tv
global TP TW TV
TP=cell(4,1);TW=cell(4,1);TV=cell(4,1);
tic=ones(4,1);tp=ones(4,1);tv=ones(4,1);
% 
%% 初始化x %% 不含求导变量
x0  = 0;
str = [];
ts  = [0.02 0];
simStateCompliance = 'UnknownSimState';

function sys=mdlDerivatives(t,x,u)


dx=0;
sys = dx;

function sys=mdlUpdate(t,x,u)
global hp hv t2 t4 t5 t6 t7 L1 L2 L3 % 周期T




if t>=t2
%     if t>t4 && t<t5 %link 1-4 failure
%         L=L3;
%     else
%         if t>t6 && t<t7 % DG 3 OFF
%             L=L2;
%         else
%             L=L1;
%         end
%     end
L=L1;
    
    if mod(t,hp)==0 
       etc_dp(u,t,L);
    end    
    %%  vlotage 
    if mod(t,hv)==0   
       etc_dv(u,t,L);    
    end
end
sys = [];

function sys=mdlOutputs(t,x,u)
global zw_kh zv_kh zp_kh
global TP TW TV Tend

if t==Tend
    save TV 
    save TW 
    save TP    
end

sys=[zw_kh(1,1),zw_kh(2,1),zw_kh(3,1),zw_kh(4,1),...
    zv_kh(1,1),zv_kh(2,1),zv_kh(3,1),zv_kh(4,1),...
    zp_kh(1,1),zp_kh(2,1),zp_kh(3,1),zp_kh(4,1)]';

% ==========================================================================

function sys=mdlGetTimeOfNextVarHit(t,x,u)
sampleTime = 1;    %  Example, set the next hit to be one second later.
sys = t + sampleTime;

function sys=mdlTerminate(t,x,u)
sys = [];

function etc_dp(u,prac_t,L)
global t4 t5 t6 t7 G  w_ref
global p_hat w_hat ci_p ETC_flagp 
global zw_kh zp_kh
global TW TP tp tic
%% 拓扑

%%
zp_kh=-L*p_hat;
pi=[u(9) u(10) u(11) u(12)]';
flagg=(pi-p_hat).*(pi-p_hat)-diag(ci_p)*(zp_kh.*zp_kh);

for i=1:4
    if i==3 && (prac_t<t7 && prac_t>t6)
        flagg(i,1)=-1;
    end
    if flagg(i,1)>0  | flagg(i,1)==0
        ETC_flagp(i,1)=1;
        p_hat(i,1)=u(8+i);
        w_hat(i,1)=u(i);  
        TP{i,1}(tp(i)+1,1:2)=[prac_t,p_hat(i,1)];
        TW{i,1}(tic(i)+1,1:2)=[prac_t,w_hat(i,1)];
        tp(i)=tp(i)+1; tic(i)=tic(i)+1; 
    else
        ETC_flagp(i,1)=0;
    end
end
zw_kh=-(L+G)*(w_hat-w_ref*ones(4,1));
zp_kh=-L*p_hat;

function etc_dv(u,prac_t,L)
global ci_v v_ref 
global G 
global  t2 t4 t5 t6 t7
global ETC_flagv zv_kh v_hat 
global TV tv

%%
zv_kh=-(L+G)*(v_hat-v_ref*ones(4,1));
vi=[u(5) u(6) u(7) u(8)]';
flagg=(vi-v_hat).*(vi-v_hat)-diag(ci_v)*(zv_kh.*zv_kh);
%%  更新驱动标志
if prac_t==t2
     v_hat=[u(5),u(6),u(7),u(8)]';
     TV={[t2,u(5)];[t2,u(6)];[t2,u(7)];[t2,u(8)]};
     tv=[2,2,2,2];
else  
    for i=1:4
        if i==1
            if abs(u(5)-v_ref)<2e-3
               flagg(i,1)=-1;
            end
        else
            if i==3
                if abs(zv_kh(i,1))<2e-3 || (prac_t<t7 && prac_t>t6)
                    flagg(i,1)=-1;
                end
            else
                if abs(zv_kh(i,1))<2e-3
                    flagg(i,1)=-1;
                end
            end
        end
        if flagg(i,1)>0 | flagg(i,1)==0
            ETC_flagv(i)=1;        
            v_hat(i,1)=u(4+i);
            TV{i,1}(tv(i)+1,1:2)=[prac_t,v_hat(i,1)];
            tv(i)=tv(i)+1;
        else
            ETC_flagv(i,1)=0;
        end        
    end
end
zv_kh=-(L+G)*(v_hat-v_ref*ones(4,1));







