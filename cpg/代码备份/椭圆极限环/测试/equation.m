%% ��������

%�˴�����ΪCPG��ģ��
function [yy]=equation(t,value,omiga)
    alpha=0.003;%x�᷽�������ٶ�
    x=value(1);%��������� x_p0 ������
    y=value(2);%��������� y_p0 ������
    u=10000; %Բ�İ뾶
    beta=0.75;  %ռ�ձ�(֧����)
    w_sw=12.5*pi;     %��Ƶ��(�ڶ���)
    b_p=50;     %����֧���ࡢ�ڶ������л��ٶȣ�����Ϊ�ܴ����0��+
    
    %����wi
    omiga=(1-beta)/beta*w_sw/(1+exp(-b_p*y))+w_sw/(1+exp(b_p*y));
    yy(1,1)=alpha*(u-x*x-y*y)*x+omiga*y;
    yy(1,2)=alpha*(u-x*x-y*y)*y-omiga*x;
    
end



%%���þ���
% [T1,F1]=runge_kutta(@equation,[0;0.2],0.01,0,10);
% plot(F1(:,1),F1(:,2),'.')