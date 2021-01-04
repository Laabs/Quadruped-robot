%% ��������

%�˴�����ΪCPG��ģ��
function [yy]=equation(t,value,phi_ij)
    alpha=0.005;%x�᷽�������ٶ�
    u=10000; %Բ�İ뾶
    beta=0.75;  %ռ�ձ�(֧����)
    w_sw=4.5*pi;     %��Ƶ��(�ڶ���)
    b_p=0;     %����֧���ࡢ�ڶ������л��ٶȣ�����Ϊ�ܴ����0��
    
    

    x1=value(1,1);%��������� x_p0 ������
    y1=value(1,2);%��������� y_p0 ������
    x2=value(2,1);%��������� x_p0 ������
    y2=value(2,2);%��������� y_p0 ������
    x3=value(3,1);%��������� x_p0 ������
    y3=value(3,2);%��������� y_p0 ������
    x4=value(4,1);%��������� x_p0 ������
    y4=value(4,2);%��������� y_p0 ������
    
    %����wi
    omiga1=(1-beta)/beta*w_sw/(1+exp(-b_p*y1))+w_sw/(1+exp(b_p*y1));
    omiga2=(1-beta)/beta*w_sw/(1+exp(-b_p*y2))+w_sw/(1+exp(b_p*y2));
    omiga3=(1-beta)/beta*w_sw/(1+exp(-b_p*y3))+w_sw/(1+exp(b_p*y3));
    omiga4=(1-beta)/beta*w_sw/(1+exp(-b_p*y4))+w_sw/(1+exp(b_p*y4));
    
    xxx=[x1,x2,x3,x4];
    yyy=[y1,y2,y3,y4];
    for j=1:1:4
        thetax(j)=0;
        thetay(j)=0;
        for k=1:1:4
%             if j~=k
                thetax(j)=thetax(j)+cos(phi_ij(j)-phi_ij(k))*xxx(k)-sin(phi_ij(j)-phi_ij(k))*yyy(k);
                thetay(j)=thetay(j)+sin(phi_ij(j)-phi_ij(k))*xxx(k)+cos(phi_ij(j)-phi_ij(k))*yyy(k);
%             end
        end
    end
    

    yy1(1,1)=alpha*(u-x1*x1-y1*y1)*x1-omiga1*y1 + thetax(1);
    yy1(1,2)=alpha*(u-x1*x1-y1*y1)*y1+omiga1*x1 +thetay(1);
    
    yy2(1,1)=alpha*(u-x2*x2-y2*y2)*x2-omiga2*y2+thetax(2);
    yy2(1,2)=alpha*(u-x2*x2-y2*y2)*y2+omiga2*x2+thetay(2);
   
    yy3(1,1)=alpha*(u-x3*x3-y3*y3)*x3-omiga3*y3+thetax(3);
    yy3(1,2)=alpha*(u-x3*x3-y3*y3)*y3+omiga3*x3+thetay(3);
    
    yy4(1,1)=alpha*(u-x4*x4-y4*y4)*x4-omiga4*y4+thetax(4);
    yy4(1,2)=alpha*(u-x4*x4-y4*y4)*y4+omiga4*x4+thetay(4);
    
    yy=[yy1;yy2;yy3;yy4];
end



%%���þ���
% [T1,F1]=runge_kutta(@equation,[0;0.2],0.01,0,10);
% plot(F1(:,1),F1(:,2),'.')



