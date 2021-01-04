%% ������������΢�ַ�����Ľ�
% ufunc:΢�ַ���������
% y0����ʼֵ
% h������
% [a,b]��ʱ�������յ�

function [t,yp,yf]=runge_kutta(equation,y0,h,a,b)
n=floor((b-a)/h);%����
t(1)=a;          %ʱ�����
yp(1,:)=y0;        %����ֵ������������������Ҫע��ά��
yf(1,:)=y0;
for ii=1:n
% ��������
ym=[yp(ii,:);yf(ii,:)];

t(ii+1)=t(ii)+h;
k1=equation(t(ii),ym);
k2=equation(t(ii)+h/2,ym+h*k1/2);
k3=equation(t(ii)+h/2,ym+h*k2/2);
k4=equation(t(ii)+h,ym+h*k3);
% yp(ii+1,:)=ym(ii,:)+h*(k1+2*k2+2*k3+k4)/6;
yp(ii+1,:)=ym(1,:)+h*(k1(1,:)+2*k2(1,:)+2*k3(1,:)+k4(1,:))/6;
yf(ii+1,:)=ym(2,:)+h*(k1(2,:)+2*k2(2,:)+2*k3(2,:)+k4(2,:))/6;

%���������������������ֵ��� 
end





