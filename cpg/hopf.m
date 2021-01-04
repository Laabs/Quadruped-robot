% �Ľ���Բ����Hopf

function dydt=hopf(t,x_y)
x=x_y(1);
y=x_y(2);
alpha=5;%x�᷽�������ٶ�
beta=10;%y�᷽�������ٶ�
a=150;%��Բ����
b=100;%��Բ����
omiga=pi;% ����Ƶ�ʣ�ԽС������Խ��


dydt = [alpha*(1-x*x/a/a-y*y/b/b)*x+omiga*a/b*y;...
    beta*(1-x*x/a/a-y*y/b/b)*y-omiga*b/a*x]; 
end