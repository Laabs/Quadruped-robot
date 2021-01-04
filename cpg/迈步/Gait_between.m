function [xx,yy]=Gait_between(t,value,phi,n,xy_j)

    alpha=5;%x�᷽�������ٶ�
    beta=10;%y�᷽�������ٶ�
    L_s=150; %��Բ���ᣬ��������
    H_s=100; %��Բ���ᣬ���̧�ȸ߶�
    
    
    b_p=20; %�ڶ����֧������л��ٶ�

    
    D_f=0.75;%ռ��ϵ��
    V_f=15;

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

    x_j=xy_j(1);%�ο���x����
    y_j=xy_j(2);%�ο���y����
    x=value(1);%��������� x_p0 ������
    y=value(2);%��������� y_p0 ������
    
    omiga=pi*V_f/L_s*(D_f/(1-D_f)/(1+exp(-b_p*y))+1/(1+exp(b_p*y)));%��Ƶ��(��)
    yy(1,1)=alpha*(1-4*x*x/L_s/L_s-y*y/H_s/H_s)*x+omiga*L_s/H_s/2*y;
    yy(1,2)=beta*(1-4*x*x/L_s/L_s-y*y/H_s/H_s)*y-omiga*2*H_s/L_s*x;
    

end