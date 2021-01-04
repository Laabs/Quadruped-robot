%% �������壨΢�ַ��̣�

%�˴�����ΪCPG��ģ��
function [yy]=equation(t,value,phi_ij,beta_stance)
    alpha=30;%x�᷽�������ٶ�
    beta=20;%y�᷽�������ٶ�
    L_s=150; %��Բ���ᣬ��������
    H_s=100; %��Բ���ᣬ���̧�ȸ߶�
    %omiga=pi;% ����Ƶ�ʣ�ԽС������Խ��
    
    b_p=60; %�ڶ����֧������л��ٶȣ��Ƚϴ�ĳ�����
    b_f=60;
    ytd=-20;

    
    D_f=beta_stance;%ռ��ϵ��0.75
    V_f=25;
    Kc=2;
    
    x1=value(1,1);%��������� x_p0 ������
    y1=value(1,2);%��������� y_p0 ������
    x2=value(3,1);%��������� x_p0 ������
    y2=value(3,2);%��������� y_p0 ������
    x3=value(5,1);%��������� x_p0 ������
    y3=value(5,2);%��������� y_p0 ������
    x4=value(7,1);%��������� x_p0 ������
    y4=value(7,2);%��������� y_p0 ������
    
    xf1=value(2,1);%��������� x_f0 ������
    yf1=value(2,2);%��������� y_f0 ������
    xf2=value(4,1);%��������� x_f0 ������
    yf2=value(4,2);%��������� y_f0 ������
    xf3=value(6,1);%��������� x_f0 ������
    yf3=value(6,2);%��������� y_f0 ������
    xf4=value(8,1);%��������� x_f0 ������
    yf4=value(8,2);%��������� y_f0 ������
    
    xxx=[x1,x2,x3,x4];
    yyy=[y1,y2,y3,y4];
        
        
    for j=1:1:4
        thetax(j)=0;
        thetay(j)=0;
        for k=1:1:4
%             if j~=k
%               thetax(j)=thetax(j)+phi_ij(1,k)*xxx(k)-phi_ij(2,k)*yyy(k);
%               thetay(j)=thetay(j)+phi_ij(2,k)*xxx(k)+phi_ij(1,k)*yyy(k);
                thetax(j)=thetax(j)+cos(phi_ij(j)-phi_ij(k))*xxx(k)-sin(phi_ij(j)-phi_ij(k))*yyy(k);
                thetay(j)=thetay(j)+sin(phi_ij(j)-phi_ij(k))*xxx(k)+cos(phi_ij(j)-phi_ij(k))*yyy(k);
%             end
        end
    end
    
    
%    y1=1;
%    y2=2;
%    y3=3;
%    y4=4;
 C=[0 -1 1 -1;-1 0 -1 1;-1 1 0 -1;1 -1 -1 0]*[y1;y2;y3;y4];

%LF1
    omiga_s=pi*V_f/L_s*(D_f/(1-D_f)/(1+exp(-b_p*y1))+1/(1+exp(b_p*y1)));%��Ƶ��(��)
    dxp1=alpha*(1-4*x1*x1/L_s/L_s-y1*y1/H_s/H_s)*x1+omiga_s*L_s/H_s/2*y1 ;
    dyp1=beta*(1-4*x1*x1/L_s/L_s-y1*y1/H_s/H_s)*y1-omiga_s*2*H_s/L_s*x1 +C(1);
%RF1
    omiga_s=pi*V_f/L_s*(D_f/(1-D_f)/(1+exp(-b_p*y2))+1/(1+exp(b_p*y2)));%��Ƶ��(��)
    dxp2=alpha*(1-4*x2*x2/L_s/L_s-y2*y2/H_s/H_s)*x2+omiga_s*L_s/H_s/2*y2;
    dyp2=beta*(1-4*x2*x2/L_s/L_s-y2*y2/H_s/H_s)*y2-omiga_s*2*H_s/L_s*x2  +C(2);
%LH2    
    omiga_s=pi*V_f/L_s*(D_f/(1-D_f)/(1+exp(-b_p*y3))+1/(1+exp(b_p*y3)));%��Ƶ��(��)
    dxp3=alpha*(1-4*x3*x3/L_s/L_s-y3*y3/H_s/H_s)*x3+omiga_s*L_s/H_s/2*y3 ;
    dyp3=beta*(1-4*x3*x3/L_s/L_s-y3*y3/H_s/H_s)*y3-omiga_s*2*H_s/L_s*x3 +C(3) ;
%RH2    
    omiga_s=pi*V_f/L_s*(D_f/(1-D_f)/(1+exp(-b_p*y4))+1/(1+exp(b_p*y4)));%��Ƶ��(��)
    dxp4=alpha*(1-4*x4*x4/L_s/L_s-y4*y4/H_s/H_s)*x4+omiga_s*L_s/H_s/2*y4 ;
    dyp4=beta*(1-4*x4*x4/L_s/L_s-y4*y4/H_s/H_s)*y4-omiga_s*2*H_s/L_s*x4  +C(4);
    
    
    
    dxf1=(dxp1+Kc*(x1-xf1))/(1+exp(-b_f*(y1-ytd)))-V_f/(1+exp(b_f*(y1-ytd)));
    dyf1=(dyp1+Kc*(y1-yf1))/(1+exp(-b_f*(y1-ytd)));
    
    
    dxf2=(dxp2+Kc*(x2-xf2))/(1+exp(-b_f*(y2-ytd)))-V_f/(1+exp(b_f*(y2-ytd)));
    dyf2=(dyp2+Kc*(y2-yf2))/(1+exp(-b_f*(y2-ytd)));
    
    
    dxf3=(dxp3+Kc*(x3-xf3))/(1+exp(-b_f*(y3-ytd)))-V_f/(1+exp(b_f*(y3-ytd)));
    dyf3=(dyp3+Kc*(y3-yf3))/(1+exp(-b_f*(y3-ytd)));
    
    
    dxf4=(dxp4+Kc*(x4-xf4))/(1+exp(-b_f*(y4-ytd)))-V_f/(1+exp(b_f*(y4-ytd)));
    dyf4=(dyp4+Kc*(y4-yf4))/(1+exp(-b_f*(y4-ytd)));
    
    yy=[dxp1,dyp1;dxf1,dyf1;dxp2,dyp2;dxf2,dyf2;dxp3,dyp3;dxf3,dyf3;dxp4,dyp4;dxf4,dyf4];
end





%%���þ���
% [T1,F1]=runge_kutta(@equation,[0;0.2],0.01,0,10);
% plot(F1(:,1),F1(:,2),'.')