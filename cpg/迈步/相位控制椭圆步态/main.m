
clc;
clear;

%-----------------------------------------------------��ز���------------------------------------
% 1.��̬����
lamda=0.25; %���ڿ���ռ�õ�ʱ��
zo=0;xo=0;zd=80;xd=150;yo=0;yd=0;
z_high=0;
%���½����λ��
zm=-440;
xm=-80;
ym=40;

T=0.4;


beta=0.5;  %ռ�ձ�(֧����)

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
P1_m=[20,-440];
P2_m=[20,-440];
P3_m=[20,-440];
P4_m=[20,-440];


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


%---------------------------------------------------VREP��ʼ�����----------------------------------
vrep=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)

vrep.simxFinish(-1); % �Ͽ�����������

while 1
    clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);
    if (clientID>-1)  
        disp('���ӳɹ�');
        
        break;    
    else
         disp('����ʧ��') ;  
    end
    
     pause(0.05);  
end
 
%----------------------------------------------------------------------------------------

%--------------------------------------------------��ȡ�ؽھ��------------------------------------
    vrep.simxAddStatusbarMessage(clientID ,'hello vrep , i am matlab ',vrep.simx_opmode_blocking);
    % ��1
    [returncode1,motor11]=vrep.simxGetObjectHandle(clientID,'motor11',vrep.simx_opmode_blocking);
    [~,motor12]=vrep.simxGetObjectHandle(clientID,'motor12',vrep.simx_opmode_blocking);
    [~,motor13]=vrep.simxGetObjectHandle(clientID,'motor13',vrep.simx_opmode_blocking);
    % ��2
    [~,motor21]=vrep.simxGetObjectHandle(clientID,'motor21',vrep.simx_opmode_blocking);
    [~,motor22]=vrep.simxGetObjectHandle(clientID,'motor22',vrep.simx_opmode_blocking);
    [~,motor23]=vrep.simxGetObjectHandle(clientID,'motor23',vrep.simx_opmode_blocking);
    % ��3
    [~,motor31]=vrep.simxGetObjectHandle(clientID,'motor31',vrep.simx_opmode_blocking);
    [~,motor32]=vrep.simxGetObjectHandle(clientID,'motor32',vrep.simx_opmode_blocking);
    [~,motor33]=vrep.simxGetObjectHandle(clientID,'motor33',vrep.simx_opmode_blocking);
    % ��4
    [~,motor41]=vrep.simxGetObjectHandle(clientID,'motor41',vrep.simx_opmode_blocking);
    [~,motor42]=vrep.simxGetObjectHandle(clientID,'motor42',vrep.simx_opmode_blocking);
    [~,motor43]=vrep.simxGetObjectHandle(clientID,'motor43',vrep.simx_opmode_blocking);
    pause(0.1);
  %-----------------------------------------------------------------------------------------    
    
  if (returncode1==vrep.simx_return_ok)
    disp('��ȡ����������');
    vrep.simxStartSimulation(clientID,vrep.simx_opmode_oneshot);
     while 1
                tstep = 0.01;  % ����ģ��Ĳ���Ϊ5ms/��,��verp�е�����Ϊһ����ʵ��ͬ������
                vrep.simxSetFloatingParameter(clientID,vrep.sim_floatparam_simulation_time_step,tstep,vrep.simx_opmode_oneshot);%����ģ�ⲽ��
                recode=vrep.simxSynchronous(clientID,true);
                if(recode==vrep.simx_return_ok)
                    disp('ʹ��ͬ���ɹ�');
    %                 vrep.simxSetJointPosition (clientID,motor1,-0.5,vrep.simx_opmode_oneshot );                   vrep.simxSetJointPosition (clientID,motor2,-0.2,vrep.simx_opmode_oneshot );
                    disp('�����ʼ��.....');
                     pause(2);
                   vrep.simxSynchronousTrigger(clientID);
                    break;
                else continue;
                end
     end

      %---------------------------------------------��ʼ����λ---------------------------
%  %1  
       disp('��ʼ����λ');
       %��ȡ���
       [~,init_angle(1)]=vrep.simxGetJointPosition(clientID,motor11,vrep.simx_opmode_streaming);
       [~,init_angle(2)]=vrep.simxGetJointPosition(clientID,motor12,vrep.simx_opmode_streaming);
       [~,init_angle(3)]=vrep.simxGetJointPosition(clientID,motor13,vrep.simx_opmode_streaming);
%  %2  
       [~,init_angle(4)]=vrep.simxGetJointPosition(clientID,motor21,vrep.simx_opmode_streaming);
       [~,init_angle(5)]=vrep.simxGetJointPosition(clientID,motor22,vrep.simx_opmode_streaming);
       [~,init_angle(6)]=vrep.simxGetJointPosition(clientID,motor23,vrep.simx_opmode_streaming);

%  %3  
       [~,init_angle(7)]=vrep.simxGetJointPosition(clientID,motor31,vrep.simx_opmode_streaming);
       [~,init_angle(8)]=vrep.simxGetJointPosition(clientID,motor32,vrep.simx_opmode_streaming);
       [~,init_angle(9)]=vrep.simxGetJointPosition(clientID,motor33,vrep.simx_opmode_streaming);
%  %4
       [~,init_angle(10)]=vrep.simxGetJointPosition(clientID,motor41,vrep.simx_opmode_streaming);
       [~,init_angle(11)]=vrep.simxGetJointPosition(clientID,motor42,vrep.simx_opmode_streaming);
       [~,init_angle(12)]=vrep.simxGetJointPosition(clientID,motor43,vrep.simx_opmode_streaming);
       
       disp('11,12,13,21,22,23,31,32,33,41,42,43');
       disp(init_angle*180/pi); 
       disp('��ʼ����λ���');
%------------------------------------------------------------------------------------------------------------------------
  

        %���ò�̬�滮����
%         [Gx,Gz]=G(xo,xd,zo,zd,z_high,lamda,zm,xm); %0��60,0,50,0,0.25,-120,0
        
        
        % i������d����
        % hdtheta=[0,0,0,0];
        % idtheta=[0,0,0,0];
        % hdphi=[0,0,0,0];
        % idphi=[0,0,0,0];

        % ��ʼλ�ò�ֵ
        angletheta=[-2.63,-2.63,-2.63,-2.63]*pi/180; % ����Ϊ�� motor1
        anglephi=[10.3,10.3,10.3,10.3]*pi/180;% ��ǰΪ�� motor2
        anglegama=[-6.15,-6.15,-6.15,-6.15]*pi/180; %����Ϊ�� motor3 -10.3


        % ������
        motor1=[motor11,motor21,motor31,motor41]; %
        motor2=[motor12,motor22,motor32,motor42]; %
        motor3=[motor13,motor23,motor33,motor43]; %
        %��ȡ�ؽ�λ�� v2 phi motor1
        for j=0:100
            for num=1:4
                [~,jointPositiontheta(num)]=vrep.simxGetJointPosition(clientID,motor1(num),vrep.simx_opmode_buffer);
                [~,jointPositionphi(num)]=vrep.simxGetJointPosition(clientID,motor2(num),vrep.simx_opmode_buffer);
                [~,jointPositiongama(num)]=vrep.simxGetJointPosition(clientID,motor3(num),vrep.simx_opmode_buffer);
            end
            disp(jointPositiontheta);
            disp(jointPositionphi);
            disp(jointPositiongama);
            pause(0.5);
            if(jointPositiontheta(1)<pi)
%                 if num==3 ||num==4
%                     jointPositiontheta=-jointPositiontheta;
%                 end    
                init_angle=[jointPositiontheta,jointPositionphi,jointPositiongama];
                break;
            end
        end

%         angle1=[0:0.05:pi/2,pi/2:-0.05:-pi/8,-pi/8:0.05:0]; % pid����ר��
        
%         ttt=1:1:length(angle1);
%         plot(ttt,angle1);
        inum=1;
        % motor1��theta��motor2��phi��motor3��gama
        num_cpg=1;
        
    while 1
       
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
% 
%          [theta(1),phi(1),gama(1)]=anglesolve_Online(P1_plot(num_cpg,1),-0,P1_plot(num_cpg,2)); %motor 1
%          [theta(4),phi(4),gama(4)]=anglesolve_Online(P2_plot(num_cpg,1),-0,P2_plot(num_cpg,2)); %motor 4
%          [theta(2),phi(2),gama(2)]=anglesolve_Online(P3_plot(num_cpg,1),0,P3_plot(num_cpg,2)); %motor 2
%          [theta(3),phi(3),gama(3)]=anglesolve_Online(P4_plot(num_cpg,1),0,P4_plot(num_cpg,2)); %motor 3
         
         [theta(3),phi(3),gama(3)]=anglesolve_Online(P1_plot(num_cpg,1),-0,P1_plot(num_cpg,2)); %motor 1
         [theta(2),phi(2),gama(2)]=anglesolve_Online(P2_plot(num_cpg,1),-0,P2_plot(num_cpg,2)); %motor 4
         [theta(4),phi(4),gama(4)]=anglesolve_Online(P3_plot(num_cpg,1),0,P3_plot(num_cpg,2)); %motor 2
         [theta(1),phi(1),gama(1)]=anglesolve_Online(P4_plot(num_cpg,1),0,P4_plot(num_cpg,2)); %motor 3
        
         
         num_cpg=num_cpg+1;
         if num_cpg==300
               beta=0.5;  %ռ�ձ�(֧����)
                % ��������λ
                phi_LF=0;
                phi_RF=0.5;
                phi_LH=beta;
                phi_RH=beta-phi_RF;
                %��������ڵ�һ���ȵ���λ��1-4(LF��RF��LH��RH)
                phi_ij=[2*pi*phi_LF,2*pi*phi_RF,2*pi*phi_LH,2*pi*phi_RH];
                disp('��̬�л���0.75-0.5��');
                
         end
%         if inum > length(angle1)  % pid���ڻ�ͼ
%             inum=1;
%             xxx=1:1:length(gama_plot1(:,1));
%             
%             % gama[1 - 4]
%             figure(1);
%             subplot(2,2,1);
%             plot(xxx,gama_plot1(:,1),'r',xxx,gama_plot2(:,1),'k');
%             
%             subplot(2,2,2);
%             plot(xxx,gama_plot1(:,2),'r',xxx,gama_plot2(:,2),'k');
%             
%             subplot(2,2,3);
%             plot(xxx,gama_plot1(:,3),'r',xxx,gama_plot2(:,3),'k');
%             
%             subplot(2,2,4);
%             plot(xxx,gama_plot1(:,4),'r',xxx,gama_plot2(:,4),'k');
%             
%             %phi[1 - 4] 
%             yyy=1:1:length(phi_plot1(:,1));
%             figure(2);
%             subplot(2,2,1);
%             plot(yyy,phi_plot1(:,1),'r',yyy,phi_plot2(:,1),'k');
%             
%             subplot(2,2,2);
%             plot(yyy,phi_plot1(:,2),'r',yyy,phi_plot2(:,2),'k');
%             
%             subplot(2,2,3);
%             plot(yyy,phi_plot1(:,3),'r',yyy,phi_plot2(:,3),'k');
%             
%             subplot(2,2,4);
%             plot(yyy,phi_plot1(:,4),'r',yyy,phi_plot2(:,4),'k');
%           
%               %theta[1 - 4]
%               figure(3);
%               zzz=1:1:length(theta_plot1(:,4));
%               subplot(2,2,1);
%               plot(xxx,theta_plot1(:,1),'r',xxx,theta_plot2(:,1),'k');
%               
%               subplot(2,2,2);
%               plot(xxx,theta_plot1(:,2),'r',xxx,theta_plot2(:,2),'k');
%               
%               subplot(2,2,3);
%               plot(xxx,theta_plot1(:,3),'r',xxx,theta_plot2(:,3),'k');
%               
%               subplot(2,2,4);
%               plot(xxx,theta_plot1(:,4),'r',xxx,theta_plot2(:,4),'k');
% 
%         end
        
            % 1.��ǰʵ�ʽǶ� ��motor1��theta��motor2��phi��motor3��gama��
            for num=1:4
                [~,jointPositiontheta(num)]=vrep.simxGetJointPosition(clientID,motor1(num),vrep.simx_opmode_buffer);
                [~,jointPositionphi(num)]=vrep.simxGetJointPosition(clientID,motor2(num),vrep.simx_opmode_buffer);
                [~,jointPositiongama(num)]=vrep.simxGetJointPosition(clientID,motor3(num),vrep.simx_opmode_buffer);
            end

%  pid ����ר��
%             theta=[-angle1(inum),-angle1(inum),angle1(inum),angle1(inum)]; %����Ϊ�����������Ϊ��
%             phi=[angle1(inum),angle1(inum),angle1(inum),angle1(inum)]; %����Ϊ�����������Ϊ��
%             gama=[angle1(inum),angle1(inum),angle1(inum),angle1(inum)]; %����Ϊ�����������Ϊ��
            
            % 3.����Ƕ�ƫ����ٶ�
            for num=1:4
              % �Ƕ�ƫ��
              dtheta=theta(num)+angletheta(num)-jointPositiontheta(num)-init_angle(num);
              dphi=phi(num)-anglephi(num)-jointPositionphi(num)-init_angle(num+4);
              dgama=gama(num)-anglegama(num)-jointPositiongama(num)-init_angle(num+8);
              
              % �ٶ�ֵ
              v1(num)=10*dtheta;%10  ��Bullet��Newton �������桿
              v2(num)=4.2*dphi;%4.2  ��Bullet��Newton �������桿
              v3(num)=6*dgama;%6     ��Bullet��Newton �������桿
              
              % ��ȡֵ��ͼ pid ����ר��
%               theta_plot1(inum,num)=theta(num);
%               theta_plot2(inum,num)=jointPositiontheta(num)-angletheta(num)+init_angle(num);
%               gama_plot1(inum,num)=gama(num);
%               gama_plot2(inum,num)=jointPositiongama(num)+anglegama(num)+init_angle(num+8);
%               phi_plot1(inum,num)=phi(num);
%               phi_plot2(inum,num)=jointPositionphi(num)+anglephi(num)+init_angle(num+4);
              
            end

            % 4.�����ٶ�
            for num=1:4
              vrep.simxSetJointTargetVelocity(clientID,motor1(num),v1(num),vrep.simx_opmode_oneshot);% ���1
              vrep.simxSetJointTargetVelocity(clientID,motor2(num),v2(num),vrep.simx_opmode_oneshot);% ���2
              vrep.simxSetJointTargetVelocity(clientID,motor3(num),v3(num),vrep.simx_opmode_oneshot);% ���3
            end
            
           inum=inum+1;
%            pause(0.01);
           vrep.simxSynchronousTrigger(clientID);
    end
    
%------------------------------------------------------------------------------------------------------------       
else 
    disp('��ȡ����������ʧ�ܣ���������˳� ')
    pause; 
end

vrep.simxStopSimulation(clientID,vrep.simx_opmode_oneshot_wait);

vrep.simxFinish(clientID);

vrep.delete(); % call the destructor!




