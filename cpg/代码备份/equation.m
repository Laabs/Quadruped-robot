%% ��������

%�˴�����ΪCPG��ģ��
function [yy]=equation(t,value)
    alpha=5;%x�᷽�������ٶ�
    beta=10;%y�᷽�������ٶ�
    L_s=150; %��Բ���ᣬ��������
    H_s=100; %��Բ���ᣬ���̧�ȸ߶�
    %omiga=pi;% ����Ƶ�ʣ�ԽС������Խ��
    
    b_p=20; %�ڶ����֧������л��ٶ�
%     b_f=20;
%     ztd=0;
%     Kc=0;
    
    D_f=0.75;%ռ��ϵ��
    V_f=15;
    % omiga=pi*V_f/L_s*(D_f/(1-D_f)*)
%     omiga_swing=pi*(1-D_f)*V_f/L_s;
%     omiga_stance=pi*D_f*V_f/L_s;

    
    x=value(1);%��������� x_p0 ������
    y=value(2);%��������� y_p0 ������
    
%     omiga=omiga_stance/(1+exp(-b_p*y))+omiga_swing/(1+exp(b_p*y));
    
    omiga=pi*V_f/L_s*(D_f/(1-D_f)/(1+exp(-b_p*y))+1/(1+exp(b_p*y)));%��Ƶ��(��)
    
    yy(1,1)=alpha*(1-4*x*x/L_s/L_s-y*y/H_s/H_s)*x+omiga*L_s/H_s/2*y;
    yy(1,2)=beta*(1-4*x*x/L_s/L_s-y*y/H_s/H_s)*y-omiga*2*H_s/L_s*x;
    
%     dxp=alpha*(1-4*x*x/L_s/L_s-y*y/H_s/H_s)*x+omiga*L_s/H_s/2*y;
%     dy=beta*(1-4*x*x/L_s/L_s-y*y/H_s/H_s)*y-omiga*2*H_s/L_s*x;
%     xf=(dxp)/(1+exp(-b_f*(dy-ztd)))-V_f/(1+exp(-b_f*(dy-ztd)));
%     
%     yy(1,1)=xf;
%     yy(1,2)=dy;
end



%%���þ���
% [T1,F1]=runge_kutta(@equation,[0;0.2],0.01,0,10);
% plot(F1(:,1),F1(:,2),'.')