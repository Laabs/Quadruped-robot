

% ufunc:΢�ַ���������
% y0����ʼֵ
% h������
% [a,b]��ʱ�������յ�

clc;
clear;

[T1,F1]=runge_kutta(@equation,[100;-80],0.01,0,100);
plot(F1(:,1),F1(:,2),'.')
hold on

% ����λ��ʱ��ͼ
% tt=0.05*(0:1:length(F1(:,1))-1);
% plot(tt,F1(:,1),'r.',tt,F1(:,2),'g.')
% xlabel('ʱ��t(ms)');
% ylabel('λ��s(cm)');
% legend('x','z');

beta=0.75;

% ��������λ
phi_LF=0;
phi_RF=0.5;
phi_LH=beta;
phi_RH=beta-phi_RF;

%��������ڵ�һ���ȵ���λ��1-4(LF��RF��LH��RH)
phi1=2*pi*phi_LF;
phi2=2*pi*phi_RF;
phi3=2*pi*phi_LH;
phi4=2*pi*phi_RH;

phi_ij=0;

% ��ͬ�ŵĲ�̬
Rphi_ij=[cos(phi_ij),-sin(phi_ij);sin(phi_ij),cos(phi_ij)];






