function dydt=hopf1(t,x_y)
x=x_y(1);
y=x_y(2);
alpha=5 ;% x�᷽�������ٶ�
omiga=pi;% ����Ƶ�ʣ�ԽС������Խ��
u=100;


dydt = [alpha*(u-x*x-y*y)*x-omiga*y;...
    alpha*(u-x*x-y*y)*y+omiga*x]; 
end