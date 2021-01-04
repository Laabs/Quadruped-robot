
yy1=[0.2,0];


alpha=5;%x�᷽�������ٶ�
beta=10;%y�᷽�������ٶ�
L_s=150;%��Բ����
H_s=100;%��Բ����
omiga=pi;% ����Ƶ�ʣ�ԽС������Խ��
b_p=2; %�ڶ����֧������л��ٶ�

D_f=0.75;
V_f=15;
% omiga=pi*V_f/L_s*(D_f/(1-D_f)*)
omiga_swing=pi*(1-D_f)*V_f/L_s;
omiga_stance=pi*D_f*V_f/L_s;

ufunc = [alpha*(1-x*x/L_s/L_s-y*y/H_s/H_s)*x+omiga*L_s/H_s*y;...
    beta*(1-x*x/L_s/L_s-y*y/H_s/H_s)*y-omiga*H_s/L_s*x]; 

[x1,y1]=runge_kutta1(ufunc,[0.2,0],h,a,b);
plot(x1(:,1),y1(:,1));
grid on;