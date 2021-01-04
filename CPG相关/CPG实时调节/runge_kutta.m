%% ������������΢�ַ�����Ľ�
% ufunc:΢�ַ���������
% y0����ʼֵ
% h������
% [a,b]��ʱ�������յ�
% ����ֵ��t ʱ�䲽��
%         y1 - y4,4��CPG������ֵ(x,y)

clear;

%������ز���
a_link=265;
b_link=75;
d_link=240;
e_link=80;
f_link=45;
g_link=sqrt(e_link*e_link-f_link*f_link);
gama_link=147/180*pi;
Q=[-f_link;0;-g_link];


beta=0.75;  %ռ�ձ�(֧����)

% ��������λ
phi_LF=0;
phi_RF=0.5;
phi_LH=beta;
phi_RH=beta-phi_RF;

%��������ڵ�һ���ȵ���λ��1-4(LF��RF��LH��RH)
phi_ij=[2*pi*phi_LF,2*pi*phi_RF,2*pi*phi_LH,2*pi*phi_RH];


% ������ʼ���꣨����������㣩
P1=[-0.2,0];
P2=[-2,0];
P3=[-4,0];
P4=[-6,0];
% ���ĵ����꣨������Źؽڣ�
P1_m=[-80,-440];
P2_m=[-80,-440];
P3_m=[-80,-440];
P4_m=[-80,-440];


P1_plot=[];
P2_plot=[];
P3_plot=[];
P4_plot=[];

% ʱ�䲽��
h=0.01;

% ��̬���
Hs=0;% y��ص���ظ�
H=80/100;%����
L=150/200;%����

num=1;
while(1)
% ����xi��yi
ym=[P1;P2;P3;P4];
% ����xi��yi
P1_plot=[P1_plot;L*P1(1)+P1_m(1),-H*P1(2)*(P1(2)<Hs)+P1_m(2)];
P2_plot=[P2_plot;L*P2(1)+P2_m(1),-H*P2(2)*(P2(2)<Hs)+P2_m(2)];
P3_plot=[P3_plot;L*P3(1)+P3_m(1),-H*P3(2)*(P3(2)<Hs)+P3_m(2)];
P4_plot=[P4_plot;L*P4(1)+P4_m(1),-H*P4(2)*(P4(2)<Hs)+P4_m(2)];
% �������������[x,y]
k1=equation(ym,phi_ij);
k2=equation(ym+h*k1/2,phi_ij);
k3=equation(ym+h*k2/2,phi_ij);
k4=equation(ym+h*k3,phi_ij);
P1=P1+h*(k1(1,:)+2*k2(1,:)+2*k3(1,:)+k4(1,:))/6;
P2=P2+h*(k1(2,:)+2*k2(2,:)+2*k3(2,:)+k4(2,:))/6;
P3=P3+h*(k1(3,:)+2*k2(3,:)+2*k3(3,:)+k4(3,:))/6;
P4=P4+h*(k1(4,:)+2*k2(4,:)+2*k3(4,:)+k4(4,:))/6;

% tt=h*(0:1:length(P1_plot(:,1))-1);
pause(0.001);
% subplot(2,2,[1,3]);
% plot(P1_plot(:,1),P1_plot(:,2),'.');
% subplot(2,2,2);
% plot(tt,P1_plot(:,1),'.',tt,P2_plot(:,1),'.',tt,P3_plot(:,1),'.',tt,P4_plot(:,1),'.');
% subplot(2,2,4);
% plot(tt,P1_plot(:,2),'.',tt,P2_plot(:,2),'.',tt,P3_plot(:,2),'.',tt,P4_plot(:,2),'.');
% 
% % ym(ii+1,:)=ym(ii,:)+h*(k1+2*k2+2*k3+k4)/6;
% if length(P1_plot(:,1))==100
%     beta=0.5;  %ռ�ձ�(֧����)
%    
%     % ��������λ
%     phi_LF=0;
%     phi_RF=0.5;
%     phi_LH=beta;
%     phi_RH=beta-phi_RF;
% 
%     %��������ڵ�һ���ȵ���λ��1-4(LF��RF��LH��RH)
%     phi_ij=[2*pi*phi_LF,2*pi*phi_RF,2*pi*phi_LH,2*pi*phi_RH];
% end

% %------------------------------ĩ���������������ϵ����--------------------------
         A_x=P1_plot(num,1);
         A_y=40;
         A_z=P1_plot(num,2);

         % 1.���˲���
        a_link=265;
        b_link=75;
        d_link=240;
        e_link=80;
        f_link=45;
        g_link=sqrt(e_link*e_link-f_link*f_link);
        gama_link=147/180*pi;
        Q=[-f_link;0;-g_link];

         
         rox=atan2(A_y,-A_z); %�����ת��1������Ϊ��������Ϊ��
         Rx=[1 0 0;0 cos(rox) -sin(rox);0 sin(rox) cos(rox)];%��ת����
         P_A=[A_x;A_y;A_z];
         P_B=Rx'*P_A; %P_A=Rx*P_B;
         x=P_B(1);
         y=P_B(2);
         z=P_B(3);
         
         QD=sqrt((x-Q(1))^2+(z-Q(3))^2);
         alpha_link=atan2(-(z-Q(3)),x-Q(1))+acos((a_link*a_link+QD*QD-d_link*d_link)/(2*a_link*QD)); %�����ת��2����Ϊ��
         C=[Q(1)+a_link*cos(alpha_link);0;Q(3)-a_link*sin(alpha_link)];
         beta_link=gama_link-acos((C(1)-x)/d_link);                         %�����ת��3������Ϊ��������Ϊ��

        A=[Q(1)-b_link*cos(beta_link);0;Q(3)+b_link*sin(beta_link)];
        B=[Q(1)+a_link*cos(alpha_link)-b_link*cos(beta_link);0;Q(3)-a_link*sin(alpha_link)+b_link*sin(beta_link)];
        D=[x;0;z];


        A=Rx*A;
        B=Rx*B;
        C=Rx*C;
        D=Rx*D;
        A_Q=Rx*Q;

        xx(num)=D(1);
        yy(num)=D(2);
        zz(num)=D(3);

        plot3([A_Q(1),0],[A_Q(2),0],[A_Q(3),0]);
        hold on;
        plot3([A_Q(1),A(1)],[A_Q(2),A(2)],[A_Q(3),A(3)]);
        hold on;
        plot3([A(1),B(1)],[A(2),B(2)],[A(3),B(3)]);
        hold on;
        plot3([B(1),C(1)],[B(2),C(2)],[B(3),C(3)]);
        hold on;
        plot3([C(1),D(1)],[C(2),D(2)],[C(3),D(3)]);
        hold on;
        plot3([A_Q(1),C(1)],[A_Q(2),C(2)],[A_Q(3),C(3)]);
        hold on;
        %%---------
        plot3([0,0],[-200,200],[0,0]);
        hold on;
        plot3([-200,200],[0,0],[0,0]);
        hold on;
        plot3([0,0],[0,0],[-200,200]);
        hold on;
        %%---------
        plot3(xx,yy,zz,'.');
        hold on;
        axis([-600,400,-600,400,-600,400])
        grid on;
        hold off;
        num=num+1;

end








