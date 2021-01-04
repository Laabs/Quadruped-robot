

% ufunc:΢�ַ���������
% y0����ʼֵ
% h������
% [a,b]��ʱ�������յ�

clc;
clear;

[T1,F1]=runge_kutta(@equation,[0;0.5],0.01,0,10);
 plot(F1(:,1),F1(:,2),'.')

[T2,F2]=runge_kutta(@equation,[0;0.5],0.01,0,5);
% plot(F2(:,1),F2(:,2),'.')

[T3,F3]=runge_kutta(@equation,[0;0.5],0.01,0,2);




tt=0.05*(0:1:length(F1(:,1))-1);
% plot(tt,F1(:,1),'r.',tt,F2(:,1),'g.',tt,F3(:,1),'b.')

subplot(3,1,1)
plot(tt,F1(:,2),'r.')
legend('beta=0.25');
subplot(3,1,2)
plot(tt,F2(:,2),'g.')
legend('beta=0.5');
subplot(3,1,3)
plot(tt,F3(:,2),'b.')
legend('beta=0.75');


xlabel('ʱ��t(ms)');
ylabel('λ��s(cm)');




legend('w=0.5pi','w=pi','w=2pi');

% ����λ��ʱ��ͼ
% tt=0.05*(0:1:length(F1(:,1))-1);
% plot(tt,F1(:,1),'r.',tt,F1(:,2),'g.')
% xlabel('ʱ��t(ms)');
% ylabel('λ��s(cm)');
% legend('x','z');


% ��������λ
phi_LF=0;
phi_RF=0.5;
phi_LH=beta;
phi_RH=beta-phi_RF;

%��������ڵ�һ���ȵ���λ��1-4(LF��RF��LH��RH)
phi_ij=[2*pi*phi_LF,2*pi*phi_RF,2*pi*phi_LH,2*pi*phi_RH];













