%% ���ٿ��� - ��̬�滮
%xx0  x�����������
%xxb  x�����յ�����
%zz0  %z�����������
%zzb  %z�����յ�����
%z_high %¥�ݸ߶�
%lamda  %������ռʱ��
%zm,xm %����λ��
%��������� h�Ӵ������������lamda�Ӵ�

function [x,z]=G(xx0,xxb,zz0,zzb,z_high,lamda,zm,xm) %0��60,0,50,0,0.25,-120,0
% 1.��ʼ������
a_xmax=250;   %x���������ٶ�
v_xmax=130;   %x��������ٶ�
s_x(1)=xx0;    %x������ʼλ��
v_x(1)=0;    %x������ʼ�ٶ�
sb_x=xxb;     %x����ǰ������
h=0.25;      %����
step=500;    %�ܲ������ֵ����


if sqrt(sb_x*a_xmax)<=v_xmax             %�ȼ��١��ȼ��� ��������ʱ���һ��
    t_mid=sqrt(sb_x/a_xmax);
else                                     %�ȼ��١����١��ȼ��� ��������ʱ���һ��
    t_mid=v_xmax/a_xmax+(sb_x/2-v_xmax*v_xmax/a_xmax/2)/v_xmax; 
end

for k=1:step
    [s1,v1,a1]=fh_avmax(sb_x,v_xmax,a_xmax,h,s_x(k),v_x(k));
    s_x(k+1)=s1;
    v_x(k+1)=v1;
    a_x(k)=a1;
    if abs(s1-sb_x)<0.5*a_xmax*h*h
        t_x=k;
        break;
    end
end

v_body=sb_x/(h*(t_x)/lamda);         %����ǰ���ٶȣ�����ʱ�������

for k=1:t_x+1
    s_x(k)=s_x(k)-(k-1)*h*v_body;          %�ȵ��ٶ� - ������ٶ�
end

%ֱ�߻���ʱ��

t_x2=h*(t_x+1):h:(s_x(t_x+1)-s_x(1))/v_body+h*(t_x+1); %ֱ�߻���
s_x2=s_x(t_x+1)-v_body*(t_x2-h*(t_x+1));

%%%%%��ز���
s1_z(1)=zz0;
v1_z(1)=0;
sb_z1=zzb;
sb2_z=s1_z(1)+z_high;

% �ȼ��١��ȼ��٣����ٶ���С��
a_zmax1=(sqrt(sb_z1)+sqrt(sb_z1-sb2_z))^2/((t_mid)^2);
v_zmax1=ceil(sqrt(2*sb_z1*a_zmax1))+5;


a_zmax2=a_zmax1;
v_zmax2=v_zmax1;
s2_z(1)=sb_z1;
v2_z(1)=0;

% ��һ������
for k=1:step
    [sh,vh,ah]=fh_avmax(sb_z1,v_zmax1,a_zmax1,h,s1_z(k),v1_z(k));
    s1_z(k+1)=sh;
    v1_z(k+1)=vh;
    if abs(sh-sb_z1)<0.005
        t_z1=k-1;
        break;
    end
end

%%�ڶ�������
s2_z(1)=s1_z(t_z1);
v2_z(1)=v1_z(t_z1);

for k=1:step
    [sh,vh,ah]=fh_avmax(sb2_z,v_zmax2,a_zmax2,h,s2_z(k),v2_z(k));
    s2_z(k+1)=sh;
    v2_z(k+1)=vh;
    if abs(sh-sb2_z)<0.005
        t_z2=k;
        break;
    end
end

s3_z=sb2_z+0*t_x2;  %�ڵ����ϵĵ�


if t_z2+t_z1>t_x
    x=s_x;
    for k=1:t_x+1
        if k<t_z1
            z(k)=s1_z(k);
        else
            z(k)=s2_z(k-t_z1+1);   
        end
    end
elseif t_z2+t_z1==t_x
      for k=1:t_x
        x(k)=s_x(k);
        if k<t_z1+1
            z(k)=s1_z(k);
        else
            z(k)=s2_z(k-t_z1+1);   
        end
      end 
else
    z=[s1_z,s2_z]; 
      for k=1:t_z1+t_z2+2
        x(k)=s_x(k);
      end       
end

x=[x,s_x2]+xm;
z=[z,s3_z]+zm;

% %�켣
% subplot(2,3,1)
% plot(x,z,'.'); 
% title('x-z');
end








