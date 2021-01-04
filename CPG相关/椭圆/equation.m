%% �������壨΢�ַ��̣�

%�˴�����ΪCPG��ģ��
function [yy]=equation(t,value)
    alpha=5;%x�᷽�������ٶ�
    beta=10;%y�᷽�������ٶ�
    L_s=150; %��Բ���ᣬ��������
    H_s=100; %��Բ���ᣬ���̧�ȸ߶�
    %omiga=pi;% ����Ƶ�ʣ�ԽС������Խ��
    
    b_p=60; %�ڶ����֧������л��ٶȣ��Ƚϴ�ĳ�����
    b_f=60;
    ytd=-20;

    
    D_f=0.75;%ռ��ϵ��
    V_f=15;
    Kc=2;
    
%     omiga=pi*V_f/L_s*(D_f/(1-D_f)*)
%     omiga_swing=pi*(1-D_f)*V_f/L_s;
%     omiga_stance=pi*D_f*V_f/L_s;

    x=value(1,1);%��������� x_p0 ������
    y=value(1,2);%��������� y_p0 ������
    omiga_s=pi*V_f/L_s*(D_f/(1-D_f)/(1+exp(-b_p*y))+1/(1+exp(b_p*y)));%��Ƶ��(��)
    dxp=alpha*(1-4*x*x/L_s/L_s-y*y/H_s/H_s)*x+omiga_s*L_s/H_s/2*y;
    dyp=beta*(1-4*x*x/L_s/L_s-y*y/H_s/H_s)*y-omiga_s*2*H_s/L_s*x;
    
    xf=value(2,1);%��������� x_p0 ������
    yf=value(2,2);%��������� y_p0 ������
    dxf=(dxp+Kc*(x-xf))/(1+exp(-b_f*(y-ytd)))-V_f/(1+exp(b_f*(y-ytd)));
    dyf=(dyp+Kc*(y-yf))/(1+exp(-b_f*(y-ytd)));
    yy=[dxp,dyp;dxf,dyf];
end



%%���þ���
% [T1,F1]=runge_kutta(@equation,[0;0.2],0.01,0,10);
% plot(F1(:,1),F1(:,2),'.')