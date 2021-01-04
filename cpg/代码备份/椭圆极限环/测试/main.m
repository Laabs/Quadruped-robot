
clc
clear

[T1,F]=runge_kutta(@equation,[0.5;0.9],0.01,0,5);

plot(F(:,1),F(:,2),'.')
xlabel('xλ��s(cm)');
ylabel('yλ��s(cm)');


% tt=0.05*(0:1:length(F1(:,1))-1);
plot(T1,F(:,1),'r',T1,F(:,2),'g')
xlabel('ʱ��t(cm)');
ylabel('λ��s(cm)');
legend('x','y');
