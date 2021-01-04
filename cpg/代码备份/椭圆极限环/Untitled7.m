

% ufunc:微分方程组名称
% y0：初始值
% h：步长
% [a,b]：时间起点和终点

clc;
clear;

[T1,F1,F2]=runge_kutta(@equation,[0.5;0],0.04,0,100);
plot(F1(:,1),F1(:,2),'.')
hold on
plot(F1(:,1),F2(:,2),'.')
hold on

% 绘制位移时间图
tt=0.05*(0:1:length(F1(:,1))-1);
plot(tt,F2(:,1),'r.',tt,F2(:,2),'g.')
xlabel('时间t(ms)');
ylabel('位移s(cm)');
legend('x','z');

beta=0.75;

% 四条腿相位
phi_LF=0;
phi_RF=0.5;
phi_LH=beta;
phi_RH=beta-phi_RF;

%各腿相对于第一条腿的相位差1-4(LF、RF、LH、RH)
phi1=2*pi*phi_LF;
phi2=2*pi*phi_RF;
phi3=2*pi*phi_LH;
phi4=2*pi*phi_RH;

phi_ij=0;

% 不同脚的步态
Rphi_ij=[cos(phi_ij),-sin(phi_ij);sin(phi_ij),cos(phi_ij)];




clc;
clear;

[T1,F1,F2,F3,F4]=runge_kutta(@equation,[0.5;0],0.02,0,80);

plot(F3(:,1),F3(:,2),'.')
hold on
plot(F3(:,1),F3(:,4),'.')
hold on

plot(T1,F1(:,1),T1,F2(:,1),T1,F3(:,1),T1,F4(:,1));






