%% ������������΢�ַ�����Ľ�
% ufunc:΢�ַ���������
% y0����ʼֵ
% h������
% [a,b]��ʱ�������յ�

function [t,y]=runge_kutta(equation,y0,h,a,b)
n=floor((b-a)/h);%����
t(1)=a;          %ʱ�����
y(1,:)=y0;        %����ֵ������������������Ҫע��ά��
for ii=1:n

t(ii+1)=t(ii)+h;
k1=equation(t(ii),y(ii,:));
k2=equation(t(ii)+h/2,y(ii,:)+h*k1/2);
k3=equation(t(ii)+h/2,y(ii,:)+h*k2/2);
k4=equation(t(ii)+h,y(ii,:)+h*k3);
y(ii+1,:)=y(ii,:)+h*(k1+2*k2+2*k3+k4)/6;
%���������������������ֵ���
end




