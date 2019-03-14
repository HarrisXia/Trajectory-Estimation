%% note:decentrailized kalman filter applied on altermeter,v_sensor and accerometer repectively;
%1����˫����GPS����̬�Ĳ�����2���������ϵ���ϵͳ����̬�Ĳ�����3����MEMS�ߵ�����̬�Ĳ���;
%�����ں�û�п��Ǵ�����֮��Ļ�Э���ֱ�ӻ���CI�㷨���е�
%%
clear
clc
%% parameter
T = 0.01;
N = 2000;              %����������
alpha1 = 0.01;
alpha2 = 0.01;
alpha3 = 0.01;
sigma_w = 0.01;       %standard deviation of system noise
sigma1 = 0.1;         %standard deviation of sensor1(GPS)
sigma2 = 0.3;         %standard deviation of sensor2(radio)
sigma3 = 1;           %standard deviation of sensor3(ADS, barometric altimeter)
I3 = eye(3);
%% variable pre_define
X0 = [0 1 176]';  % initial state of the system
X_p11 = zeros(3,N);
X_p21 = zeros(3,N);
X_p31 = zeros(3,N);
X_m = zeros(3,N);
X11_dKalman = zeros(3,N);
X21_dKalman = zeros(3,N);
X31_dKalman = zeros(3,N);
e11 = zeros(3,N);
e21 = zeros(3,N);
e31 = zeros(3,N);
y11 = zeros(3,N);
y21 = zeros(3,N);
y31 = zeros(3,N);
X_p11(:,1) = X0;
X_p21(:,1) = X0;
X_p31(:,1) = X0;
X_m(:,1) = X0;
X_exp = zeros(3,N);
y_exp = zeros(3,N);
X11_dKalman(:,1) = X0;
X21_dKalman(:,1) = X0;
X31_dKalman(:,1) = X0;
roll_fusion_CI = zeros(1,N);
pitch_fusion_CI = zeros(1,N);
yaw_fusion_CI = zeros(1,N);
roll_fusion_CI(1) = X0(1);
pitch_fusion_CI(1) = X0(2);
yaw_fusion_CI(1) = X0(3);
bias_roll_fusion_CI = zeros(1,N);
bias_pitch_fusion_CI = zeros(1,N);
bias_yaw_fusion_CI = zeros(1,N);
a1 = 1/3;  %%%%%%CI�㷨�ں�ϵ��,3sensors have the same eiificient
a2 = 1/3;
a3 = 1/3;
roll_fusion_CI1 = zeros(1,N);
roll_fusion_CI1(1) = X0(1);
bias_fusion_CI1 = zeros(1,N);
b1 = 1/2;   %%%%%%CI�㷨�ں�ϵ��,3sensors have different dfficient regarding their covariance
b2 = 1/3;
b3 = 1/6;
%% noise
w = normrnd(0,sigma_w,[3,N]);
eta1 = normrnd(0,sigma1,[3,N]);
eta2 = normrnd(0,sigma2,[3,N]);
eta3 = normrnd(0,sigma3,[3,N]);
v11 = 0.1*w+3*eta1;

% v2���һ��ϡ��֮��İ����������Բ�ֵ��
v21 = zeros(3,N);
v2_m = normrnd(0,sigma2,[3,200]);
v21(:,1:200) = v2_m;
for i = 200:-1:2
    v21(:,10*i-9) = v21(:,i);
    de = (v21(:,10*i-9)-v21(:,i-1))/3;
    if (de>=0)
        v21(:,10*i-10) = v21(:,10*i-9)-de;
        v21(:,10*i-11) = v21(:,10*i-9)-2*de;
        v21(:,10*i-12) = v21(:,10*i-9)-3*de;
        v21(:,10*i-13) = v21(:,10*i-9)-4*de;
        v21(:,10*i-14) = v21(:,10*i-9)-5*de;
        v21(:,10*i-15) = v21(:,10*i-9)-6*de;
        v21(:,10*i-16) = v21(:,10*i-9)-7*de;
        v21(:,10*i-17) = v21(:,10*i-9)-8*de;
        v21(:,10*i-18) = v21(:,10*i-9)-9*de;
    else 
        v21(:,10*i-10) = v21(:,10*i-9)+de;
        v21(:,10*i-11) = v21(:,10*i-9)+2*de;
        v21(:,10*i-12) = v21(:,10*i-9)+3*de;
        v21(:,10*i-13) = v21(:,10*i-9)+4*de;
        v21(:,10*i-14) = v21(:,10*i-9)+5*de;
        v21(:,10*i-15) = v21(:,10*i-9)+6*de;
        v21(:,10*i-16) = v21(:,10*i-9)+7*de;
        v21(:,10*i-17) = v21(:,10*i-9)+8*de;
        v21(:,10*i-18) = v21(:,10*i-9)+9*de;
    end
end

v31 = 0.1*w+eta3;
% ������ͳ�����ԣ�Q��w�ķ��S��w��v'�ĳ˻�����ѧ������R��v�ķ���
trace_Q = [sigma_w^2 sigma_w^2 sigma_w^2];
Q = diag(trace_Q);
trace_S11 = [0.01*sigma_w^2,0.01*sigma_w^2,0.01*sigma_w^2];
S11 = diag(trace_S11);    % S1 = E(w*v1')=E(w*(0.005w'+eta1'))=E(0.005*w*w')
S21 = zeros(3,3);
trace_S31 = [0.1*sigma_w^2,0.1*sigma_w^2,0.1*sigma_w^2];
S31 = diag(trace_S31);

trace_R11 = [0.01*sigma_w^2+sigma1^2,0.01*sigma_w^2+sigma1^2,0.01*sigma_w^2+sigma1^2];
R11 = diag(trace_R11);
trace_R21 = [0.4,0.4,0.4];   %�����˼���ʵ��ͳ�ƣ������0.4����
R21 = diag(trace_R21);  
trace_R31 = [0.01*sigma_w^2+sigma3^2,0.01*sigma_w^2+sigma3^2,0.01*sigma_w^2+sigma3^2];
R31 = diag(trace_R31);
%% system matrix
A = I3; %x(k+1) = A*x(k)+B*u(k)+gamma*w(k);y1(k)=H1(k)*x(k)+v1(k)
B = I3;
gamma = I3;
H = I3;
J11 = gamma*S11/R11;
J21 = gamma*S21/R21;
J31 = gamma*S31/R31;
deltaA_bar11 = A-J11*H;
deltaA_bar21 = A-J21*H;
deltaA_bar31 = A-J31*H;
P11_p = 0.1*I3;
P11 = P11_p;
P21 = P11_p;
P31 = P11_p;
%% �����������
u = zeros(3,N);
%�����Ǻ����������ƫ����
for k = 501:510
    u(1,k) = -0.08;
end
for k = 1501:1510
    u(1,k) = 0.18;
end
for k = 301:310
    u(3,k) = 3.8;
end
for k = 1001:1020
    u(3,k) = 2.6;
end
for k = 1501:1520
    u(3,k) = 4.7;
end
%% system simulation  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
X_exp(:,1) = X0;
y_exp(:,1) = H*X_m(:,1);
for k = 2:N
    X_exp(:,k) = A*X_exp(:,k-1)+B*u(:,k-1)+gamma*w(:,k-1);
    for i=1:3
        if(X_exp(i,k)>180)
            X_exp(i,k) = X_exp(i,k)-360;
        elseif(X_exp(i,k)<-180)
            X_exp(i,k) = X_exp(i,k)+360;   
        end
    end
    y_exp(:,k) = H*X_exp(:,k);  %�۲�ֵ 
end
roll_exp = y_exp(1,:);
pitch_exp = y_exp(2,:);
yaw_exp = y_exp(3,:);

%% sensor1,2,3 system simulation
y11(:,1) = H*X_m(:,1)+v11(:,1);
for k = 2:N
    X_m(:,k) = A*X_m(:,k-1)+B*u(:,k)+gamma*w(:,k-1);%ģ��ֵ����ֵ��
    y11(:,k) = H*X_m(:,k)+v11(:,k);  %�۲�ֵ 
end
y21(:,1) = H*X_m(:,1)+v21(:,1);
for k = 2:N
    y21(:,k) = H*X_m(:,k)+v21(:,k);  %�۲�ֵ,״̬���̲���Ҫ�ٵ���һ�Σ��ٴε�������Ϊ��˹������������ԣ�����ϵͳ����ֵ���Ĳ�һ�£�
end
y31(:,1) = H*X_m(:,1)+v31(:,1);
for k = 2:N
    y31(:,k) = H*X_m(:,k)+v31(:,k);  %�۲�ֵ,״̬���̲���Ҫ�ٵ���һ�Σ��ٴε�������Ϊ��˹������������ԣ�����ϵͳ����ֵ���Ĳ�һ�£�
end

%% yaw fusino, filtering makes worse result than take the measurement directly
var_y11 = 0.01^3+0.09;
var_y21 = 0.4;
var_y31 = 0.01^3+1;
%% sensor1,2,3 decentralized kalman filter
for k =2:N  
   %% kalman filter
%     X_p11(:,k) = A*X11_dKalman(:,k-1)+B*u(:,k-1);
%     X_p21(:,k) = A*X21_dKalman(:,k-1)+B*u(:,k-1);
%     X_p31(:,k) = A*X31_dKalman(:,k-1)+B*u(:,k-1);
    X_p11(:,k) = deltaA_bar11*X11_dKalman(:,k-1)+B*u(:,k-1)+J11*y11(:,k-1);
    X_p21(:,k) = deltaA_bar21*X21_dKalman(:,k-1)+B*u(:,k-1)+J21*y21(:,k-1);
    X_p31(:,k) = deltaA_bar31*X31_dKalman(:,k-1)+B*u(:,k-1)+J31*y31(:,k-1);
%     P11_p = A*P11*A'+Q;
%     P21_p = A*P21*A'+Q;
%     P31_p = A*P31*A'+Q;
    P11_p = deltaA_bar11*P11*deltaA_bar11'+gamma*(Q-S11/R11*S11')*gamma';
    P21_p = deltaA_bar21*P21*deltaA_bar21'+gamma*(Q-S21/R21*S21')*gamma';
    P31_p = deltaA_bar31*P31*deltaA_bar31'+gamma*(Q-S31/R31*S31')*gamma';
    K11_g = P11_p*H'/(H*P11_p*H'+R11);
    K21_g = P21_p*H'/(H*P21_p*H'+R21);
    K31_g = P31_p*H'/(H*P31_p*H'+R31);
    P11 = (I3-K11_g*H)*P11_p;
    P21 = (I3-K21_g*H)*P21_p;
    P31 = (I3-K31_g*H)*P31_p;
    e11(:,k) = y11(:,k)-H*X_p11(:,k);
    e21(:,k) = y21(:,k)-H*X_p21(:,k);
    e31(:,k) = y31(:,k)-H*X_p31(:,k); 
    X11_dKalman(:,k) = X_p11(:,k)+K11_g*e11(:,k);%�ֲ��˲�ֵ
    X21_dKalman(:,k) = X_p21(:,k)+K21_g*e21(:,k);%�ֲ��˲�ֵ
    X31_dKalman(:,k) = X_p31(:,k)+K31_g*e31(:,k);%�ֲ��˲�ֵ
   % **************optimal fusion based on covarianceintersection************
    w11 = P21(1,1)*P31(1,1);
    w21 = P11(1,1)*P31(1,1);
    w31 = P11(1,1)*P21(1,1);
    roll_fusion_CI(k) = (a1*w11*X11_dKalman(1,k)+a2*w21*X21_dKalman(1,k)+a3*w31*X31_dKalman(1,k))/(a1*w11+a2*w21+a3*w31);
    pitch_fusion_CI(k) = (a1*w11*X11_dKalman(2,k)+a2*w21*X21_dKalman(2,k)+a3*w31*X31_dKalman(2,k))/(a1*w11+a2*w21+a3*w31);
    yaw_fusion_CI(k) = (a1*w11*X11_dKalman(3,k)+a2*w21*X21_dKalman(3,k)+a3*w31*X31_dKalman(3,k))/(a1*w11+a2*w21+a3*w31);
    var_fusion_CI = P11(1,1)*P21(1,1)*P31(1,1)/(a1*w11+a2*w21+a3*w31);   %%1/1/3(1/P1(1,1)+1/P2(1,1)+1/P3(1,1))%%
end
%% ����ĺ����˲�ֵ��û�б����ã��˲����ܺܺõظ�����ʵֵ
roll_GPS_dKalman = X11_dKalman(1,:);
pitch_GPS_dKalman = X11_dKalman(2,:);
yaw_GPS_dKalman = X11_dKalman(3,:);
roll_OF_dKalman = X21_dKalman(1,:);
pitch_OF_dKalman = X21_dKalman(2,:);
yaw_OF_dKalman = X21_dKalman(3,:);
roll_MEMS_dKalman = X31_dKalman(1,:);
pitch_MEMS_dKalman = X31_dKalman(2,:);
yaw_MEMS_dKalman = X31_dKalman(3,:);
for k = 1:N
    if(yaw_GPS_dKalman(k)>180)
        yaw_GPS_dKalman(k) = yaw_GPS_dKalman(k) - 360;
    elseif(yaw_GPS_dKalman(k)<-180)
        yaw_GPS_dKalman(k) = yaw_GPS_dKalman(k) + 360;
    end
    if(yaw_OF_dKalman(k)>180)
        yaw_OF_dKalman(k) = yaw_OF_dKalman(k) - 360;
    elseif(yaw_OF_dKalman(k)<-180)
        yaw_OF_dKalman(k) = yaw_OF_dKalman(k) + 360;
    end
    if(yaw_MEMS_dKalman(k)>180)
        yaw_MEMS_dKalman(k) = yaw_MEMS_dKalman(k) - 360;
    elseif(yaw_MEMS_dKalman(k)<-180)
        yaw_MEMS_dKalman(k) = yaw_MEMS_dKalman(k) + 360;
    end
end
%% figure
roll_GPS_measure = y11(1,:);    %GPS stands for dual antenna GPS
pitch_GPS_measure = y11(2,:);
yaw_GPS_measure = y11(3,:);
roll_OF_measure = y21(1,:);    % OF stands for optical fiber based IMU����������˵�Ĺ�����ϵ�����
pitch_OF_measure = y21(2,:);
yaw_OF_measure = y21(3,:);
roll_MEMS_measure = y31(1,:);   % MEMS stands for MEMS IMU
pitch_MEMS_measure = y31(2,:);
yaw_MEMS_measure = y31(3,:);
% ����ȡֵ-180��180��ת��ȡֵ
for k = 1:N
    if(yaw_GPS_measure(k)>180)
        yaw_GPS_measure(k) = yaw_GPS_measure(k) - 360;
    elseif(yaw_GPS_measure(k)<-180)
        yaw_GPS_measure(k) = yaw_GPS_measure(k) + 360;
    end
    if(yaw_OF_measure(k)>180)
        yaw_OF_measure(k) = yaw_OF_measure(k) - 360;
    elseif(yaw_OF_measure(k)<-180)
        yaw_OF_measure(k) = yaw_OF_measure(k) + 360;
    end
    if(yaw_MEMS_measure(k)>180)
        yaw_MEMS_measure(k) = yaw_MEMS_measure(k) - 360;
    elseif(yaw_MEMS_measure(k)<-180)
        yaw_MEMS_measure(k) = yaw_MEMS_measure(k) + 360;
    end
    if(yaw_fusion_CI(k)>180)
        yaw_fusion_CI(k) = yaw_fusion_CI(k) - 360;
    elseif(yaw_fusion_CI(k)<-180)
        yaw_fusion_CI(k) = yaw_fusion_CI(k) + 360;
    end
end

t = 1:N;
%%
bias = zeros(1,N);
bias(1000:N) = 2*ones(1,N-1000+1);
roll_MEMS_measure_fault = roll_MEMS_measure+bias;

figure(1)
plot(t,roll_MEMS_measure_fault,'-b',t,roll_MEMS_measure_fault,'-r',t,roll_exp,'-k');
legend('sensor1��������µĺ���ǹ۲���','sensor1��������µĺ���ǹ۲���','���������ֵ');
xlabel('ʱ��/0.01s');
ylabel('�Ƕ�/��');
title('sensor1����ǹ���״̬�µĹ۲����Ա�');

%% ����1000��ʼ��sensor1���Ϻ���ںϽ����ԭ�ںϽ���Ա�
sensor1_fault_roll_fusion_CI = roll_fusion_CI;
sensor1_fualt_pitch_fusion_CI = pitch_fusion_CI;
sensor1_fualt_yaw_fusion_CI = yaw_fusion_CI;
Xp2 = X_p21;
Xp3 = X_p31;
e2 = e21;
e3 = e31;
X2_dKalman = X21_dKalman;
X3_dKalman = X31_dKalman;
P2 = P21;
P3 = P31;
for k =1000:N
    Xp2(:,k) = deltaA_bar21*X2_dKalman(:,k-1)+B*u(:,k-1)+J21*y21(:,k-1);
    Xp3(:,k) = deltaA_bar31*X3_dKalman(:,k-1)+B*u(:,k-1)+J31*y31(:,k-1);
    P2p = deltaA_bar21*P2*deltaA_bar21'+gamma*(Q-S21/R21*S21')*gamma';
    P3p = deltaA_bar31*P3*deltaA_bar31'+gamma*(Q-S31/R31*S31')*gamma';
    K2g = P2p*H'/(H*P2p*H'+R21);
    K3g = P3p*H'/(H*P3p*H'+R31);
    P2 = (I3-K2g*H)*P2p;
    P3 = (I3-K3g*H)*P3p;
    e2(:,k) = y21(:,k)-H*Xp2(:,k);
    e3(:,k) = y31(:,k)-H*Xp3(:,k); 
    X2_dKalman(:,k) = Xp2(:,k)+K2g*e2(:,k);%�ֲ��˲�ֵ
    X3_dKalman(:,k) = Xp3(:,k)+K3g*e3(:,k);%�ֲ��˲�ֵ
    w2 = P31(1,1);
    w3 = P21(1,1);
    sensor1_fault_roll_fusion_CI(k) = (0.5*w2*X2_dKalman(1,k)+0.5*w3*X3_dKalman(1,k))/(0.5*w2+0.5*w3);
    sensor1_fualt_pitch_fusion_CI(k) = (0.5*w2*X2_dKalman(2,k)+0.5*w3*X3_dKalman(2,k))/(0.5*w2+0.5*w3);
    sensor1_fualt_yaw_fusion_CI(k) = (0.5*w2*X2_dKalman(3,k)+0.5*w3*X3_dKalman(3,k))/(0.5*w2+0.5*w3);
end
figure(2)
plot(t,roll_MEMS_measure_fault,'-r',t,sensor1_fault_roll_fusion_CI,'-b',t,roll_exp,'-k');
legend('sensor1�޹����º���ǹ۲���','sensor1�����º���ǵ������ںϽ��','���������ֵ');
xlabel('ʱ��/0.01s');
ylabel('�Ƕ�/��');
title('sensor1�����µĺ���������ں���������ݶԱ�');
figure(3)
plot(t,roll_fusion_CI,'-r',t,sensor1_fault_roll_fusion_CI,'-b',t,roll_exp,'-k');
legend('sensor1�޹����º���������ںϽ��','sensor1�����º���ǵ������ںϽ��','���������ֵ');
xlabel('ʱ��/0.01s');
ylabel('�Ƕ�/��');
title('sensor1�����µĺ���������ں����޹����������ںϽ���Ա�');

%% 
figure(1)
plot(t,roll_MEMS_measure,'-b',t,roll_exp,'-k');
legend('sensor1����ǹ۲�ֵ','���������ֵ');
xlabel('ʱ��/0.01s');
ylabel('�Ƕ�/��');
title('sensor1���ں���ǵĹ۲�Ա�');

figure(2)
plot(t,roll_OF_measure,'-g',t,roll_exp,'-k');
legend('sensor2����ǹ۲�ֵ','���������ֵ');
xlabel('ʱ��/0.01s');
ylabel('�Ƕ�/��');
title('sensor2���ں���ǵĹ۲�Ա�');

figure(3)
plot(t,roll_GPS_measure,'-r',t,roll_exp,'-k');
legend('sensor3����ǹ۲�ֵ','���������ֵ');
xlabel('ʱ��/0.01s');
ylabel('�Ƕ�/��');
title('sensor3���ں���ǵĹ۲�Ա�');

figure(4)
plot(t,pitch_MEMS_measure,'-b',t,pitch_exp,'-k');
legend('sensor1�����ǹ۲�ֵ','����������ֵ');
xlabel('ʱ��/0.01s');
ylabel('�Ƕ�/��');
title('sensor1���ڸ����ǵĹ۲�Ա�');
figure(5)
plot(t,pitch_OF_measure,'-g',t,pitch_exp,'-k');
legend('sensor2�����ǹ۲�ֵ','����������ֵ');
xlabel('ʱ��/0.01s');
ylabel('�Ƕ�/��');
title('sensor2���ڸ����ǵĹ۲�Ա�');
figure(6)
plot(t,pitch_GPS_measure,'-r',t,pitch_exp,'-k');
legend('sensor3�����ǹ۲�ֵ','����������ֵ');
xlabel('ʱ��/0.01s');
ylabel('�Ƕ�/��');
title('sensor3���ڸ����ǵĹ۲�Ա�');

figure(7)
plot(t,yaw_MEMS_measure,'-b',t,yaw_exp,'-k');
legend('sensor1ƫ���ǹ۲�ֵ','ƫ��������ֵ');
xlabel('ʱ��/0.01s');
ylabel('�Ƕ�/��');
title('sensor1����ƫ���ǵĹ۲�Ա�');
figure(8)
plot(t,yaw_OF_measure,'-g',t,yaw_exp,'-k');
legend('sensor1ƫ���ǹ۲�ֵ','ƫ��������ֵ');
xlabel('ʱ��/0.01s');
ylabel('�Ƕ�/��');
title('sensor1����ƫ���ǵĹ۲�Ա�');
figure(9)
plot(t,yaw_GPS_measure,'-r',t,yaw_exp,'-k');
legend('sensor1ƫ���ǹ۲�ֵ','ƫ��������ֵ');
xlabel('ʱ��/0.01s');
ylabel('�Ƕ�/��');
title('sensor1����ƫ���ǵĹ۲�Ա�');



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure(4)
plot(t,roll_GPS_measure,'-g',t,roll_GPS_dKalman,'-r',t,roll_exp,'-k');
legend('˫����GPS����ǹ۲�ֵ','˫����GPS����ǵ��˲�ֵ','����ǵ�����ֵ');
xlabel('ʱ��/0.01s');
ylabel('�Ƕ�/��');
title('˫����GPS������˲����');

figure(5)
plot(t,pitch_GPS_measure,'-g',t,pitch_GPS_dKalman,'-r',t,pitch_exp,'-k');
legend('˫����GPS�����ǹ۲�ֵ','˫����GPS�����ǵ��˲�ֵ','�����ǵ�����ֵ');
xlabel('ʱ��/0.01s');
ylabel('�Ƕ�/��');
title('˫����GPS�������˲����');

figure(6)
plot(t,yaw_GPS_measure,'-g',t,yaw_GPS_dKalman,'-r',t,yaw_exp,'-k');
legend('˫����GPSƫ���ǹ۲�ֵ','˫����GPSƫ���ǵ��˲�ֵ','ƫ���ǵ�����ֵ');
xlabel('ʱ��/0.01s');
ylabel('�Ƕ�/��');
title('˫����GPSƫ�����˲����');

figure(7)
plot(t,roll_OF_measure,'-g',t,roll_OF_dKalman,'-r',t,roll_exp,'-k');
legend('������ϵ�������ǹ۲�ֵ','������ϵ�������ǵ��˲�ֵ','����ǵ�����ֵ');
xlabel('ʱ��/0.01s');
ylabel('�Ƕ�/��');
title('������ϵ���������˲����');

figure(8)
plot(t,pitch_OF_measure,'-g',t,pitch_OF_dKalman,'-r',t,pitch_exp,'-k');
legend('������ϵ��������ǹ۲�ֵ','������ϵ��������ǵ��˲�ֵ','�����ǵ�����ֵ');
xlabel('ʱ��/0.01s');
ylabel('�Ƕ�/��');
title('������ϵ����������˲����');

figure(9)
plot(t,yaw_OF_measure,'-g',t,yaw_OF_dKalman,'-r',t,yaw_exp,'-k');
legend('������ϵ���ƫ���ǹ۲�ֵ','������ϵ���ƫ���ǵ��˲�ֵ','ƫ���ǵ�����ֵ');
xlabel('ʱ��/0.01s');
ylabel('�Ƕ�/��');
title('������ϵ���ƫ�����˲����');

figure(11)
plot(t,roll_MEMS_measure,'-g',t,roll_MEMS_dKalman,'-r',t,roll_exp,'-k');
legend('MEMS�ߵ���Ԫ����ǹ۲�ֵ','MEMS�ߵ���Ԫ����ǵ��˲�ֵ','����ǵ�����ֵ');
xlabel('ʱ��/0.01s');
ylabel('�Ƕ�/��');
title('MEMS�ߵ���Ԫ������˲����');

figure(12)
plot(t,pitch_MEMS_measure,'-g',t,pitch_MEMS_dKalman,'-r',t,pitch_exp,'-k');
legend('MEMS�ߵ���Ԫ�����ǹ۲�ֵ','MEMS�ߵ���Ԫ�����ǵ��˲�ֵ','�����ǵ�����ֵ');
xlabel('ʱ��/0.01s');
ylabel('�Ƕ�/��');
title('MEMS�ߵ���Ԫ�������˲����');

figure(13)
plot(t,yaw_MEMS_measure,'-g',t,yaw_MEMS_dKalman,'-r',t,yaw_exp,'-k');
legend('MEMS�ߵ���Ԫƫ���ǹ۲�ֵ','MEMS�ߵ���Ԫƫ���ǵ��˲�ֵ','ƫ���ǵ�����ֵ');
xlabel('ʱ��/0.01s');
ylabel('�Ƕ�/��');
title('MEMS�ߵ���Ԫƫ�����˲����');

figure(14)
plot(t,roll_exp,'-k',t,roll_GPS_dKalman,'-g',t,roll_fusion_CI,'-r');
legend('���������ֵ','˫����GPS������˲����','����������ںϽ��');
xlabel('ʱ��/0.01s')
ylabel('�Ƕ�/��');
title('�޹�������º���ǵ������ںϽ��');

figure(15)
plot(t,pitch_exp,'-k',t,pitch_GPS_dKalman,'-g',t,pitch_fusion_CI,'-r');
legend('����������ֵ','˫����GPS�������˲����','�����������ںϽ��');
xlabel('ʱ��/0.01s')
ylabel('�Ƕ�/��');
title('�޹�������¸����ǵ������ںϽ��');

figure(16)
plot(t,yaw_exp,'-k',t,yaw_GPS_dKalman,'-g',t,yaw_fusion_CI,'-r');
legend('ƫ��������ֵ','˫����GPSƫ�����˲����','ƫ���������ںϽ��');
xlabel('ʱ��/0.01s')
ylabel('�Ƕ�/��');
title('�޹��������ƫ���ǵ������ںϽ��');
