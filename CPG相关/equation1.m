%% ��������

%�˴�����ΪCPG��ģ��
function [yy]=equation(t,value,omiga)
    alpha=0.001;%x�᷽�������ٶ�
    x=value(1);%��������� x_p0 ������
    y=value(2);%��������� y_p0 ������
    u=10000; %Բ�İ뾶


    yy(1,1)=alpha*(u-x*x-y*y)*x-omiga*y;
    yy(1,2)=alpha*(u-x*x-y*y)*y+omiga*x;
    
end



%%���þ���
% [T1,F1]=runge_kutta(@equation,[0;0.2],0.01,0,10);
% plot(F1(:,1),F1(:,2),'.')