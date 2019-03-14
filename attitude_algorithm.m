T=0.02;%步长
a=0;b=60*10;%起始时间与截止时间
t=a:T:b;%仿真时间
num=length(t);%四元数，四元数值0.02秒更新一次
t1=a:T/2:b;
num2=length(t1);%存储更新频率为0.01秒的数据

%俯仰角、偏航角、滚转角初始化,0.02秒更新一次
f=zeros(1,num);f(1)=0;%在这里设置了俯仰角、滚转角、偏航角的初值
g=zeros(1,num);g(1)=0;
p=zeros(1,num);p(1)=0;


%纬度、经度、高度的初始化，0.02秒更新一次
lat=zeros(1,num);lat(1)=4.2062;%纬度、经度、高度初值的设定
long=zeros(1,num);long(1)=0.6109;%老师给的说明有误，按excel中数据为准
h=zeros(1,num);h(1)=0;

%速度初始化，0.02秒更新一次
vnenx=zeros(1,num);vnenx(1)=0;%初始速度设定
vneny=zeros(1,num);vneny(1)=0;
vnenz=zeros(1,num);vnenz(1)=0;

%四元数初始化及初值的确定
q0=zeros(1,num);
q1=zeros(1,num);
q2=zeros(1,num);
q3=zeros(1,num);
q0(1)=cos(p(1)/2)*cos(f(1)/2)*cos(g(1)/2)-sin(p(1)/2)*sin(f(1)/2)*sin(g(1)/2);
q1(1)=cos(p(1)/2)*sin(f(1)/2)*cos(g(1)/2)-sin(p(1)/2)*cos(f(1)/2)*sin(g(1)/2);
q2(1)=cos(p(1)/2)*cos(f(1)/2)*sin(g(1)/2)+sin(p(1)/2)*sin(f(1)/2)*cos(g(1)/2);
q3(1)=cos(p(1)/2)*sin(f(1)/2)*sin(g(1)/2)+sin(p(1)/2)*cos(f(1)/2)*cos(g(1)/2);
Q=sqrt(q0(1)^2+q1(1)^2+q2(1)^2+q3(1)^2);
q0(1)=q0(1)/Q;%四元数初值的归一化处理
q1(1)=q1(1)/Q;
q2(1)=q2(1)/Q;
q3(1)=q3(1)/Q;

%wnie,wnen,wbnb初值的确定
wie=7.29211506e-005;
data1=xlsread('IMU.xlsx');%读入陀螺仪数据
data1=data1';
wbibx=data1(4,:);wbiby=data1(5,:);wbibz=data1(6,:);%陀螺仪的各个方向的输出
wbnbx=zeros(1,num2);wbnby=zeros(1,num2);wbnbz=zeros(1,num2);
wniex=zeros(1,num2);wniey=zeros(1,num2);wniez=zeros(1,num2);%wniex始终为0
wnenx=zeros(1,num2);wneny=zeros(1,num2);wnenz=zeros(1,num2);
wninx=zeros(1,num2);wniny=zeros(1,num2);wninz=zeros(1,num2);
wniey(1)=wie*cos(lat(1));wniez(1)=wie*sin(lat(1));%wnie初值的确定
wniey(2)=wniey(1);wniez(2)=wniez(1);%纬度更新较慢
wniey(3)=wniey(2);wniez(3)=wniez(2);
wnenx(1)=0;wneny(1)=0;wnenz(1)=0;%wnen初值的确定
wnenx(2)=wnenx(1);wneny(2)=wneny(1);wnenz(2)=wnenz(1);%速度更新较慢
wnenx(3)=wnenx(2);wneny(3)=wneny(2);wnenz(3)=wnenz(2);
wninx(1)=wniex(1)+wnenx(1);wniny(1)=wniey(1)+wneny(1);wninz(1)=wniez(1)+wnenz(1);
wninx(2)=wniex(2)+wnenx(2);wniny(2)=wniey(2)+wneny(2);wninz(2)=wniez(2)+wnenz(2);
wninx(3)=wniex(3)+wnenx(3);wniny(3)=wniey(3)+wneny(3);wninz(3)=wniez(3)+wnenz(3);
c00=q0(1)^2+q1(1)^2-q2(1)^2-q3(1)^2;%由四元数初值确定导航系到弹体系的转换矩阵的初值
c01=2*(q1(1)*q2(1)+q0(1)*q3(1));
c02=2*(q1(1)*q3(1)-q0(1)*q2(1));
c10=2*(q1(1)*q2(1)-q0(1)*q3(1));
c11=q0(1)^2+q2(1)^2-q1(1)^2-q3(1)^2;
c12=2*(q2(1)*q3(1)+q0(1)*q1(1));
c20=2*(q1(1)*q3(1)+q0(1)*q2(1));
c21=2*(q2(1)*q3(1)-q0(1)*q1(1));
c22=q0(1)^2+q3(1)^2-q1(1)^2-q2(1)^2;
wbinx(1)=c00*wninx(1)+c01*wniny(1)+c02*wninz(1);
wbiny(1)=c10*wninx(1)+c11*wniny(1)+c12*wninz(1);
wbinz(1)=c20*wninx(1)+c21*wniny(1)+c22*wninz(1);
wbnbx(1)=wbibx(1)-wbinx(1);
wbnby(1)=wbiby(1)-wbiny(1);
wbnbz(1)=wbibz(1)-wbinz(1);
wbnbx(2)=wbibx(2)-wbinx(1);%wbib更新快于wbin更新速度
wbnby(2)=wbiby(2)-wbiny(1);
wbnbz(2)=wbibz(2)-wbinz(1);
wbnbx(3)=wbibx(3)-wbinx(1);
wbnby(3)=wbiby(3)-wbiny(1);
wbnbz(3)=wbibz(3)-wbinz(1);

% 欧拉角变化率首先转化为机体系b相对于导航系n的角速度wbnb
% 地球相对惯性空间的转动引起的角速度wbin
% wnin = wnie + wnen
% wnie地球自转角速度(或地球相对于惯性系的角速度)在导航系中的投影
% wnen飞机相对地球的旋转角速度在导航系中的投影
% 导航系相对于惯性系的角速度在导航系中的投影


%fnib初值的确定
% data2=xlsread('IMU.xlsx');%读入加速度计的数据
% data2=data2';
fbibx=data1(1,:);fbiby=data1(2,:);fbibz=data1(3,:);%加速度计的输出
fnibx=zeros(1,num2);fniby=zeros(1,num2);fnibz=zeros(1,num2);
T00=c00;T01=c10;T02=c20;T10=c01;T11=c11;T12=c21;T20=c02;T21=c12;T22=c22;
fnibx(1)=c00*fbibx(1)+c10*fbiby(1)+c20*fbibz(1);
fniby(1)=c01*fbibx(1)+c11*fbiby(1)+c21*fbibz(1);
fnibz(1)=c02*fbibx(1)+c12*fbiby(1)+c22*fbibz(1);


%g、Re确定，在这里认为g为常值
g0=9.7803698;
Re=6378137;


for k=1:num-1  %t(k)即为当前时刻

%解四元数微分方程
K11=-0.5*wbnbx(2*k-1)*q1(k)-0.5*wbnby(2*k-1)*q2(k)-0.5*wbnbz(2*k-1)*q3(k);
K21=0.5*wbnbx(2*k-1)*q0(k)+0.5*wbnbz(2*k-1)*q2(k)-0.5*wbnby(2*k-1)*q3(k);
K31=0.5*wbnby(2*k-1)*q0(k)-0.5*wbnbz(2*k-1)*q1(k)+0.5*wbnbx(2*k-1)*q3(k);
K41=0.5*wbnbz(2*k-1)*q0(k)+0.5*wbnby(2*k-1)*q1(k)-0.5*wbnbx(2*k-1)*q2(k);
K12=-0.5*wbnbx(2*k)*(q1(k)+T*K21/2)-0.5*wbnby(2*k)*(q2(k)+T*K31/2)-0.5*wbnbz(2*k)*(q3(k)+T*K41/2);
K22=0.5*wbnbx(2*k)*(q0(k)+T*K11/2)+0.5*wbnbz(2*k)*(q2(k)+T*K31/2)-0.5*wbnby(2*k)*(q3(k)+T*K41/2);
K32=0.5*wbnby(2*k)*(q0(k)+T*K11/2)-0.5*wbnbz(2*k)*(q1(k)+T*K21/2)+0.5*wbnbx(2*k)*(q3(k)+T*K41/2);
K42=0.5*wbnbz(2*k)*(q0(k)+T*K11/2)+0.5*wbnby(2*k)*(q1(k)+T*K21/2)-0.5*wbnbx(2*k)*(q2(k)+T*K31/2);
K13=-0.5*wbnbx(2*k)*(q1(k)+T*K22/2)-0.5*wbnby(2*k)*(q2(k)+T*K32/2)-0.5*wbnbz(2*k)*(q3(k)+T*K42/2);
K23=0.5*wbnbx(2*k)*(q0(k)+T*K12/2)+0.5*wbnbz(2*k)*(q2(k)+T*K32/2)-0.5*wbnby(2*k)*(q3(k)+T*K42/2);
K33=0.5*wbnby(2*k)*(q0(k)+T*K12/2)-0.5*wbnbz(2*k)*(q1(k)+T*K22/2)+0.5*wbnbx(2*k)*(q3(k)+T*K42/2);
K43=0.5*wbnbz(2*k)*(q0(k)+T*K12/2)+0.5*wbnby(2*k)*(q1(k)+T*K22/2)-0.5*wbnbx(2*k)*(q2(k)+T*K32/2);
K14=-0.5*wbnbx(2*k+1)*(q1(k)+T*K23)-0.5*wbnby(2*k+1)*(q2(k)+T*K33)-0.5*wbnbz(2*k+1)*(q3(k)+T*K43);
K24=0.5*wbnbx(2*k+1)*(q0(k)+T*K13)+0.5*wbnbz(2*k+1)*(q2(k)+T*K33)-0.5*wbnby(2*k+1)*(q3(k)+T*K43);
K34=0.5*wbnby(2*k+1)*(q0(k)+T*K13)-0.5*wbnbz(2*k+1)*(q1(k)+T*K23)+0.5*wbnbx(2*k+1)*(q3(k)+T*K43);
K44=0.5*wbnbz(2*k+1)*(q0(k)+T*K13)+0.5*wbnby(2*k+1)*(q1(k)+T*K23)-0.5*wbnbx(2*k+1)*(q2(k)+T*K33);
q0(k+1)=q0(k)+(K11+2*K12+2*K13+K14)*T/6;
q1(k+1)=q1(k)+(K21+2*K22+2*K23+K24)*T/6;
q2(k+1)=q2(k)+(K31+2*K32+2*K33+K34)*T/6;
q3(k+1)=q3(k)+(K41+2*K42+2*K43+K44)*T/6;
Q=sqrt(q0(k+1)^2+q1(k+1)^2+q2(k+1)^2+q3(k+1)^2);
q0(k+1)=q0(k+1)/Q;
q1(k+1)=q1(k+1)/Q;
q2(k+1)=q2(k+1)/Q;
q3(k+1)=q3(k+1)/Q;%四元数的归一化处理

%矩阵T的更新，每0.02秒更新一次
BT00=T00;BT01=T01;BT02=T02;%T更新前先保存上一时刻的值
BT10=T10;BT11=T11;BT12=T12;
BT20=T20;BT21=T21;BT22=T22;
T00=q0(k+1)^2+q1(k+1)^2-q2(k+1)^2-q3(k+1)^2;
T01=2*(q1(k+1)*q2(k+1)-q0(k+1)*q3(k+1));
T02=2*(q1(k+1)*q3(k+1)+q0(k+1)*q2(k+1));
T10=2*(q1(k+1)*q2(k+1)+q0(k+1)*q3(k+1));
T11=q0(k+1)^2+q2(k+1)^2-q1(k+1)^2-q3(k+1)^2;
T12=2*(q2(k+1)*q3(k+1)-q0(k+1)*q1(k+1));
T20=2*(q1(k+1)*q3(k+1)-q0(k+1)*q2(k+1));
T21=2*(q2(k+1)*q3(k+1)+q0(k+1)*q1(k+1));
T22=q0(k+1)^2+q3(k+1)^2-q1(k+1)^2-q2(k+1)^2;

%姿态角的解算，每0.02秒更新一次
f(k+1)=asin(T21);
if(T22>0) 
    g(k+1)=atan(-T20/T22);
else if(T20<0) 
        g(k+1)=atan(-T20/T22)+pi;
    else
        g(k+1)=atan(-T20/T22)-pi;
    end
end
if(T11>0) 
    p(k+1)=atan(-T01/T11);
else if(T01>0) 
        p(k+1)=atan(-T01/T11)+pi;
    else
        p(k+1)=atan(-T01/T11)-pi;
    end
end

%fnib的更新
fnibx(2*k)=(BT00+T00)/2*fbibx(2*k)+(BT01+T01)/2*fbiby(2*k)+(BT02+T02)/2*fbibz(2*k);
fniby(2*k)=(BT10+T10)/2*fbibx(2*k)+(BT11+T11)/2*fbiby(2*k)+(BT12+T12)/2*fbibz(2*k);
fnibz(2*k)=(BT20+T20)/2*fbibx(2*k)+(BT21+T21)/2*fbiby(2*k)+(BT22+T22)/2*fbibz(2*k);
fnibx(2*k+1)=T00*fbibx(2*k+1)+T01*fbiby(2*k+1)+T02*fbibz(2*k+1);
fniby(2*k+1)=T10*fbibx(2*k+1)+T11*fbiby(2*k+1)+T12*fbibz(2*k+1);
fnibz(2*k+1)=T20*fbibx(2*k+1)+T21*fbiby(2*k+1)+T22*fbibz(2*k+1);

%速度更新，0.02秒更新一次,速度微分方程为非线性微分方程
M11=fnibx(2*k-1)+(2*wie*sin(lat(k))+vnenx(k)*tan(lat(k))/(Re+h(k)))*vneny(k)-(2*wie*cos(lat(k))+vnenx(k)/(Re+h(k)))*vnenz(k);
M21=fniby(2*k-1)-(2*wie*sin(lat(k))+vnenx(k)*tan(lat(k))/(Re+h(k)))*vnenx(k)-vneny(k)*vnenz(k)/(Re+h(k));
M31=fnibz(2*k-1)+(2*wie*cos(lat(k))+vnenx(k)/(Re+h(k)))*vnenx(k)+vneny(k)^2/(Re+h(k))-g0;
M12=fnibx(2*k)+(2*wie*sin(lat(k))+(vnenx(k)+T*M11/2)*tan(lat(k))/(Re+h(k)))*(vneny(k)+T*M21/2)-(2*wie*cos(lat(k))+(vnenx(k)+T*M11/2)/(Re+h(k)))*(vnenz(k)+T*M31/2);
M22=fniby(2*k)-(2*wie*sin(lat(k))+(vnenx(k)+T*M11/2)*tan(lat(k))/(Re+h(k)))*(vnenx(k)+T*M11/2)-(vneny(k)+T*M21/2)*(vnenz(k)+T*M31/2)/(Re+h(k));
M32=fnibz(2*k)+(2*wie*cos(lat(k))+(vnenx(k)+T*M11/2)/(Re+h(k)))*(vnenx(k)+T*M11/2)+(vneny(k)+T*M21/2)^2/(Re+h(k))-g0;
M13=fnibx(2*k)+(2*wie*sin(lat(k))+(vnenx(k)+T*M12/2)*tan(lat(k))/(Re+h(k)))*(vneny(k)+T*M22/2)-(2*wie*cos(lat(k))+(vnenx(k)+T*M12/2)/(Re+h(k)))*(vnenz(k)+T*M32/2);
M23=fniby(2*k)-(2*wie*sin(lat(k))+(vnenx(k)+T*M12/2)*tan(lat(k))/(Re+h(k)))*(vnenx(k)+T*M12/2)-(vneny(k)+T*M22/2)*(vnenz(k)+T*M32/2)/(Re+h(k));
M33=fnibz(2*k)+(2*wie*cos(lat(k))+(vnenx(k)+T*M12/2)/(Re+h(k)))*(vnenx(k)+T*M12/2)+(vneny(k)+T*M22/2)^2/(Re+h(k))-g0;
M14=fnibx(2*k+1)+(2*wie*sin(lat(k))+(vnenx(k)+T*M13)*tan(lat(k))/(Re+h(k)))*(vneny(k)+T*M23)-(2*wie*cos(lat(k))+(vnenx(k)+T*M13)/(Re+h(k)))*(vnenz(k)+T*M33);
M24=fniby(2*k+1)-(2*wie*sin(lat(k))+(vnenx(k)+T*M13)*tan(lat(k))/(Re+h(k)))*(vnenx(k)+T*M13)-(vneny(k)+T*M23)*(vnenz(k)+T*M33)/(Re+h(k));
M34=fnibz(2*k+1)+(2*wie*cos(lat(k))+(vnenx(k)+T*M13)/(Re+h(k)))*(vnenx(k)+T*M13)+(vneny(k)+T*M23)^2/(Re+h(k))-g0;
vnenx(k+1)=vnenx(k)+(M11+2*M12+2*M13+M14)*T/6;
vneny(k+1)=vneny(k)+(M21+2*M22+2*M23+M24)*T/6;
vnenz(k+1)=vnenz(k)+(M31+2*M32+2*M33+M34)*T/6;

%位置的解算,0.02秒更新一次
N11=vnenx(k)/(Re+h(k))/cos(lat(k));
N21=vneny(k)/(Re+h(k));
N31=vnenz(k);
N12=(vnenx(k)+vnenx(k+1))/2/(Re+h(k)+T*N31/2)/cos(lat(k)+T*N21/2);
N22=(vneny(k)+vneny(k+1))/2/(Re+h(k)+T*N31/2);
N32=(vnenz(k)+vnenz(k+1))/2;
N13=(vnenx(k)+vnenx(k+1))/2/(Re+h(k)+T*N32/2)/cos(lat(k)+T*N22/2);
N23=(vneny(k)+vneny(k+1))/2/(Re+h(k)+T*N32/2);
N33=(vnenz(k)+vnenz(k+1))/2;
N14=vnenx(k+1)/(Re+h(k)+T*N33)/cos(lat(k)+T*N23);
N24=vneny(k+1)/(Re+h(k)+T*N33);
N34=vnenz(k+1);
long(k+1)=long(k)+(N11+2*N12+2*N13+N14)*T/6;
lat(k+1)=lat(k)+(N21+2*N22+2*N23+N24)*T/6;
h(k+1)=h(k)+(N31+2*N32+2*N33+N34)*T/6;


%wnie,wnen,wbnb的更新
if(k==(num-1))
    wnenx(2*k+2)=wnenx(2*k+1);wneny(2*k+2)=wneny(2*k+1);wnenz(2*k+2)=wnenz(2*k+1);
    wniex(2*k+2)=wniex(2*k+1);wniey(2*k+2)=wniey(2*k+1);wniez(2*k+2)=wniez(2*k+1);
    wbibx(2*k+2)=wbibx(2*k+1);wbiby(2*k+2)=wbiby(2*k+1);wbibz(2*k+2)=wbibz(2*k+1);
    wnenx(2*k+3)=wnenx(2*k+2);wneny(2*k+3)=wneny(2*k+2);wnenz(2*k+3)=wnenz(2*k+2);
    wniex(2*k+3)=wniex(2*k+2);wniey(2*k+3)=wniey(2*k+2);wniez(2*k+3)=wniez(2*k+2);
    wbibx(2*k+3)=wbibx(2*k+2);wbiby(2*k+3)=wbiby(2*k+2);wbibz(2*k+3)=wbibz(2*k+2);
end
wniey(2*k+1)=wie*cos(lat(k+1));wniez(2*k+1)=wie*sin(lat(k+1));
wniey(2*k+2)=wniey(2*k+1);wniez(2*k+2)=wniez(2*k+1);
wniey(2*k+3)=wniey(2*k+2);wniez(2*k+3)=wniez(2*k+2);
wnenx(2*k+1)=-vneny(k+1)/(Re+h(k+1));wneny(2*k+1)=vnenx(k+1)/(Re+h(k+1));wnenz(2*k+1)=vnenx(k+1)*tan(lat(k+1))/(Re+h(k+1));
wnenx(2*k+2)=wnenx(2*k+1);wneny(2*k+2)=wneny(2*k+1);wnenz(2*k+2)=wnenz(2*k+1);
wnenx(2*k+3)=wnenx(2*k+2);wneny(2*k+3)=wneny(2*k+2);wnenz(2*k+3)=wnenz(2*k+2);
wninx(2*k+1)=wniex(2*k+1)+wnenx(2*k+1);wniny(2*k+1)=wniey(2*k+1)+wneny(2*k+1);wninz(2*k+1)=wniez(2*k+1)+wnenz(2*k+1);
wninx(2*k+2)=wniex(2*k+2)+wnenx(2*k+2);wniny(2*k+2)=wniey(2*k+2)+wneny(2*k+2);wninz(2*k+2)=wniez(2*k+2)+wnenz(2*k+2);
wninx(2*k+3)=wniex(2*k+3)+wnenx(2*k+3);wniny(2*k+3)=wniey(2*k+3)+wneny(2*k+3);wninz(2*k+3)=wniez(2*k+3)+wnenz(2*k+3);
wbinx(2*k+1)=T00*wninx(2*k+1)+T10*wniny(2*k+1)+T20*wninz(2*k+1);
wbiny(2*k+1)=T01*wninx(2*k+1)+T11*wniny(2*k+1)+T21*wninz(2*k+1);
wbinz(2*k+1)=T02*wninx(2*k+1)+T12*wniny(2*k+1)+T22*wninz(2*k+1);
wbinx(2*k+2)=T00*wninx(2*k+2)+T10*wniny(2*k+2)+T20*wninz(2*k+2);
wbiny(2*k+2)=T01*wninx(2*k+2)+T11*wniny(2*k+2)+T21*wninz(2*k+2);
wbinz(2*k+2)=T02*wninx(2*k+2)+T12*wniny(2*k+2)+T22*wninz(2*k+2);
wbinx(2*k+3)=T00*wninx(2*k+3)+T10*wniny(2*k+3)+T20*wninz(2*k+3);
wbiny(2*k+3)=T01*wninx(2*k+3)+T11*wniny(2*k+3)+T21*wninz(2*k+3);
wbinz(2*k+3)=T02*wninx(2*k+3)+T12*wniny(2*k+3)+T22*wninz(2*k+3);
wbnbx(2*k+1)=wbibx(2*k+1)-wbinx(2*k+1);
wbnby(2*k+1)=wbiby(2*k+1)-wbiny(2*k+1);
wbnbz(2*k+1)=wbibz(2*k+1)-wbinz(2*k+1);
wbnbx(2*k+2)=wbibx(2*k+2)-wbinx(2*k+2);
wbnby(2*k+2)=wbiby(2*k+2)-wbiny(2*k+2);
wbnbz(2*k+2)=wbibz(2*k+2)-wbinz(2*k+2);
wbnbx(2*k+3)=wbibx(2*k+3)-wbinx(2*k+3);
wbnby(2*k+3)=wbiby(2*k+3)-wbiny(2*k+3);
wbnbz(2*k+3)=wbibz(2*k+3)-wbinz(2*k+3);

%以增加运算量为代价来减小误差

%wbnb的更新
wniey(2*k-1)=wie*cos(lat(k));wniez(2*k-1)=wie*sin(lat(k));
wniey(2*k)=wie*cos((lat(k)+lat(k+1))/2);wniez(2*k)=wie*sin((lat(k)+lat(k+1))/2);
wniey(2*k+1)=wie*cos(lat(k+1));wniez(2*k+1)=wie*sin(lat(k+1));
wnenx(2*k-1)=-vneny(k)/(Re+h(k));wneny(2*k-1)=vnenx(k)/(Re+h(k));wnenz(2*k-1)=vnenx(k)*tan(lat(k))/(Re+h(k));
wnenx(2*k)=-(vneny(k)+vneny(k+1))/2/(Re+(h(k)+h(k+1))/2);wneny(2*k)=(vnenx(k)+vnenx(k+1))/2/(Re+(h(k)+h(k+1))/2);wnenz(2*k)=(vnenx(k)+vnenx(k+1))/2*tan((lat(k)+lat(k+1))/2)/(Re+(h(k)+h(k+1))/2);
wnenx(2*k+1)=-vneny(k+1)/(Re+h(k+1));wneny(2*k+1)=vnenx(k+1)/(Re+h(k+1));wnenz(2*k+1)=vnenx(k+1)*tan(lat(k+1))/(Re+h(k+1));
wninx(2*k-1)=wniex(2*k-1)+wnenx(2*k-1);wniny(2*k-1)=wniey(2*k-1)+wneny(2*k-1);wninz(2*k-1)=wniez(2*k-1)+wnenz(2*k-1);
wninx(2*k)=wniex(2*k)+wnenx(2*k);wniny(2*k)=wniey(2*k)+wneny(2*k);wninz(2*k)=wniez(2*k)+wnenz(2*k);
wninx(2*k+1)=wniex(2*k+1)+wnenx(2*k+1);wniny(2*k+1)=wniey(2*k+1)+wneny(2*k+1);wninz(2*k+1)=wniez(2*k+1)+wnenz(2*k+1);
wbinx(2*k-1)=BT00*wninx(2*k-1)+BT10*wniny(2*k-1)+BT20*wninz(2*k-1);
wbiny(2*k-1)=BT01*wninx(2*k-1)+BT11*wniny(2*k-1)+BT21*wninz(2*k-1);
wbinz(2*k-1)=BT02*wninx(2*k-1)+BT12*wniny(2*k-1)+BT22*wninz(2*k-1);
wbinx(2*k)=(BT00+T00)/2*wninx(2*k)+(BT10+T10)/2*wniny(2*k)+(BT20+T20)/2*wninz(2*k);
wbiny(2*k)=(BT01+T01)/2*wninx(2*k)+(BT11+T11)/2*wniny(2*k)+(BT21+T21)/2*wninz(2*k);
wbinz(2*k)=(BT02+T02)/2*wninx(2*k)+(BT12+T12)/2*wniny(2*k)+(BT22+T22)/2*wninz(2*k);
wbinx(2*k+1)=T00*wninx(2*k+1)+T10*wniny(2*k+1)+T20*wninz(2*k+1);
wbiny(2*k+1)=T01*wninx(2*k+1)+T11*wniny(2*k+1)+T21*wninz(2*k+1);
wbinz(2*k+1)=T02*wninx(2*k+1)+T12*wniny(2*k+1)+T22*wninz(2*k+1);
wbnbx(2*k-1)=wbibx(2*k-1)-wbinx(2*k-1);
wbnby(2*k-1)=wbiby(2*k-1)-wbiny(2*k-1);
wbnbz(2*k-1)=wbibz(2*k-1)-wbinz(2*k-1);
wbnbx(2*k)=wbibx(2*k)-wbinx(2*k);
wbnby(2*k)=wbiby(2*k)-wbiny(2*k);
wbnbz(2*k)=wbibz(2*k)-wbinz(2*k);
wbnbx(2*k+1)=wbibx(2*k+1)-wbinx(2*k+1);
wbnby(2*k+1)=wbiby(2*k+1)-wbiny(2*k+1);
wbnbz(2*k+1)=wbibz(2*k+1)-wbinz(2*k+1);

%解四元数微分方程
K11=-0.5*wbnbx(2*k-1)*q1(k)-0.5*wbnby(2*k-1)*q2(k)-0.5*wbnbz(2*k-1)*q3(k);
K21=0.5*wbnbx(2*k-1)*q0(k)+0.5*wbnbz(2*k-1)*q2(k)-0.5*wbnby(2*k-1)*q3(k);
K31=0.5*wbnby(2*k-1)*q0(k)-0.5*wbnbz(2*k-1)*q1(k)+0.5*wbnbx(2*k-1)*q3(k);
K41=0.5*wbnbz(2*k-1)*q0(k)+0.5*wbnby(2*k-1)*q1(k)-0.5*wbnbx(2*k-1)*q2(k);
K12=-0.5*wbnbx(2*k)*(q1(k)+T*K21/2)-0.5*wbnby(2*k)*(q2(k)+T*K31/2)-0.5*wbnbz(2*k)*(q3(k)+T*K41/2);
K22=0.5*wbnbx(2*k)*(q0(k)+T*K11/2)+0.5*wbnbz(2*k)*(q2(k)+T*K31/2)-0.5*wbnby(2*k)*(q3(k)+T*K41/2);
K32=0.5*wbnby(2*k)*(q0(k)+T*K11/2)-0.5*wbnbz(2*k)*(q1(k)+T*K21/2)+0.5*wbnbx(2*k)*(q3(k)+T*K41/2);
K42=0.5*wbnbz(2*k)*(q0(k)+T*K11/2)+0.5*wbnby(2*k)*(q1(k)+T*K21/2)-0.5*wbnbx(2*k)*(q2(k)+T*K31/2);
K13=-0.5*wbnbx(2*k)*(q1(k)+T*K22/2)-0.5*wbnby(2*k)*(q2(k)+T*K32/2)-0.5*wbnbz(2*k)*(q3(k)+T*K42/2);
K23=0.5*wbnbx(2*k)*(q0(k)+T*K12/2)+0.5*wbnbz(2*k)*(q2(k)+T*K32/2)-0.5*wbnby(2*k)*(q3(k)+T*K42/2);
K33=0.5*wbnby(2*k)*(q0(k)+T*K12/2)-0.5*wbnbz(2*k)*(q1(k)+T*K22/2)+0.5*wbnbx(2*k)*(q3(k)+T*K42/2);
K43=0.5*wbnbz(2*k)*(q0(k)+T*K12/2)+0.5*wbnby(2*k)*(q1(k)+T*K22/2)-0.5*wbnbx(2*k)*(q2(k)+T*K32/2);
K14=-0.5*wbnbx(2*k+1)*(q1(k)+T*K23)-0.5*wbnby(2*k+1)*(q2(k)+T*K33)-0.5*wbnbz(2*k+1)*(q3(k)+T*K43);
K24=0.5*wbnbx(2*k+1)*(q0(k)+T*K13)+0.5*wbnbz(2*k+1)*(q2(k)+T*K33)-0.5*wbnby(2*k+1)*(q3(k)+T*K43);
K34=0.5*wbnby(2*k+1)*(q0(k)+T*K13)-0.5*wbnbz(2*k+1)*(q1(k)+T*K23)+0.5*wbnbx(2*k+1)*(q3(k)+T*K43);
K44=0.5*wbnbz(2*k+1)*(q0(k)+T*K13)+0.5*wbnby(2*k+1)*(q1(k)+T*K23)-0.5*wbnbx(2*k+1)*(q2(k)+T*K33);
q0(k+1)=q0(k)+(K11+2*K12+2*K13+K14)*T/6;
q1(k+1)=q1(k)+(K21+2*K22+2*K23+K24)*T/6;
q2(k+1)=q2(k)+(K31+2*K32+2*K33+K34)*T/6;
q3(k+1)=q3(k)+(K41+2*K42+2*K43+K44)*T/6;
Q=sqrt(q0(k+1)^2+q1(k+1)^2+q2(k+1)^2+q3(k+1)^2);
q0(k+1)=q0(k+1)/Q;
q1(k+1)=q1(k+1)/Q;
q2(k+1)=q2(k+1)/Q;
q3(k+1)=q3(k+1)/Q;%四元数的归一化处理

%矩阵T的更新，每0.02秒更新一次
%BT保存了k时刻的T
T00=q0(k+1)^2+q1(k+1)^2-q2(k+1)^2-q3(k+1)^2;
T01=2*(q1(k+1)*q2(k+1)-q0(k+1)*q3(k+1));
T02=2*(q1(k+1)*q3(k+1)+q0(k+1)*q2(k+1));
T10=2*(q1(k+1)*q2(k+1)+q0(k+1)*q3(k+1));
T11=q0(k+1)^2+q2(k+1)^2-q1(k+1)^2-q3(k+1)^2;
T12=2*(q2(k+1)*q3(k+1)-q0(k+1)*q1(k+1));
T20=2*(q1(k+1)*q3(k+1)-q0(k+1)*q2(k+1));
T21=2*(q2(k+1)*q3(k+1)+q0(k+1)*q1(k+1));
T22=q0(k+1)^2+q3(k+1)^2-q1(k+1)^2-q2(k+1)^2;

%姿态角的解算，每0.02秒更新一次
f(k+1)=asin(T21);
if(T22>0) 
    g(k+1)=atan(-T20/T22);
else if(T20<0) 
        g(k+1)=atan(-T20/T22)+pi;
    else
        g(k+1)=atan(-T20/T22)-pi;
    end
end
if(T11>0) 
    p(k+1)=atan(-T01/T11);
else if(T01>0) 
        p(k+1)=atan(-T01/T11)+pi;
    else
        p(k+1)=atan(-T01/T11)-pi;
    end
end

%fnib的更新
fnibx(2*k-1)=BT00*fbibx(2*k-1)+BT01*fbiby(2*k-1)+BT02*fbibz(2*k-1);
fniby(2*k-1)=BT10*fbibx(2*k-1)+BT11*fbiby(2*k-1)+BT12*fbibz(2*k-1);
fnibz(2*k-1)=BT20*fbibx(2*k-1)+BT21*fbiby(2*k-1)+BT22*fbibz(2*k-1);
fnibx(2*k)=(BT00+T00)/2*fbibx(2*k)+(BT01+T01)/2*fbiby(2*k)+(BT02+T02)/2*fbibz(2*k);
fniby(2*k)=(BT10+T10)/2*fbibx(2*k)+(BT11+T11)/2*fbiby(2*k)+(BT12+T12)/2*fbibz(2*k);
fnibz(2*k)=(BT20+T20)/2*fbibx(2*k)+(BT21+T21)/2*fbiby(2*k)+(BT22+T22)/2*fbibz(2*k);
fnibx(2*k+1)=T00*fbibx(2*k+1)+T01*fbiby(2*k+1)+T02*fbibz(2*k+1);
fniby(2*k+1)=T10*fbibx(2*k+1)+T11*fbiby(2*k+1)+T12*fbibz(2*k+1);
fnibz(2*k+1)=T20*fbibx(2*k+1)+T21*fbiby(2*k+1)+T22*fbibz(2*k+1);

%速度更新，0.02秒更新一次,速度微分方程为非线性微分方程
M11=fnibx(2*k-1)+(2*wie*sin(lat(k))+vnenx(k)*tan(lat(k))/(Re+h(k)))*vneny(k)-(2*wie*cos(lat(k))+vnenx(k)/(Re+h(k)))*vnenz(k);
M21=fniby(2*k-1)-(2*wie*sin(lat(k))+vnenx(k)*tan(lat(k))/(Re+h(k)))*vnenx(k)-vneny(k)*vnenz(k)/(Re+h(k));
M31=fnibz(2*k-1)+(2*wie*cos(lat(k))+vnenx(k)/(Re+h(k)))*vnenx(k)+vneny(k)^2/(Re+h(k))-g0;
M12=fnibx(2*k)+(2*wie*sin((lat(k)+lat(k+1))/2)+(vnenx(k)+T*M11/2)*tan((lat(k)+lat(k+1))/2)/(Re+(h(k)+h(k+1))/2))*(vneny(k)+T*M21/2)-(2*wie*cos((lat(k)+lat(k+1))/2)+(vnenx(k)+T*M11/2)/(Re+(h(k)+h(k+1))/2))*(vnenz(k)+T*M31/2);
M22=fniby(2*k)-(2*wie*sin((lat(k)+lat(k+1))/2)+(vnenx(k)+T*M11/2)*tan((lat(k)+lat(k+1))/2)/(Re+(h(k)+h(k+1))/2))*(vnenx(k)+T*M11/2)-(vneny(k)+T*M21/2)*(vnenz(k)+T*M31/2)/(Re+(h(k)+h(k+1))/2);
M32=fnibz(2*k)+(2*wie*cos((lat(k)+lat(k+1))/2)+(vnenx(k)+T*M11/2)/(Re+(h(k)+h(k+1))/2))*(vnenx(k)+T*M11/2)+(vneny(k)+T*M21/2)^2/(Re+(h(k)+h(k+1))/2)-g0;
M13=fnibx(2*k)+(2*wie*sin((lat(k)+lat(k+1))/2)+(vnenx(k)+T*M12/2)*tan((lat(k)+lat(k+1))/2)/(Re+(h(k)+h(k+1))/2))*(vneny(k)+T*M22/2)-(2*wie*cos((lat(k)+lat(k+1))/2)+(vnenx(k)+T*M12/2)/(Re+(h(k)+h(k+1))/2))*(vnenz(k)+T*M32/2);
M23=fniby(2*k)-(2*wie*sin((lat(k)+lat(k+1))/2)+(vnenx(k)+T*M12/2)*tan((lat(k)+lat(k+1))/2)/(Re+(h(k)+h(k+1))/2))*(vnenx(k)+T*M12/2)-(vneny(k)+T*M22/2)*(vnenz(k)+T*M32/2)/(Re+(h(k)+h(k+1))/2);
M33=fnibz(2*k)+(2*wie*cos((lat(k)+lat(k+1))/2)+(vnenx(k)+T*M12/2)/(Re+(h(k)+h(k+1))/2))*(vnenx(k)+T*M12/2)+(vneny(k)+T*M22/2)^2/(Re+(h(k)+h(k+1))/2)-g0;
M14=fnibx(2*k+1)+(2*wie*sin(lat(k+1))+(vnenx(k)+T*M13)*tan(lat(k+1))/(Re+h(k+1)))*(vneny(k)+T*M23)-(2*wie*cos(lat(k+1))+(vnenx(k)+T*M13)/(Re+h(k+1)))*(vnenz(k)+T*M33);
M24=fniby(2*k+1)-(2*wie*sin(lat(k+1))+(vnenx(k)+T*M13)*tan(lat(k+1))/(Re+h(k+1)))*(vnenx(k)+T*M13)-(vneny(k)+T*M23)*(vnenz(k)+T*M33)/(Re+h(k+1));
M34=fnibz(2*k+1)+(2*wie*cos(lat(k+1))+(vnenx(k)+T*M13)/(Re+h(k+1)))*(vnenx(k)+T*M13)+(vneny(k)+T*M23)^2/(Re+h(k+1))-g0;
vnenx(k+1)=vnenx(k)+(M11+2*M12+2*M13+M14)*T/6;
vneny(k+1)=vneny(k)+(M21+2*M22+2*M23+M24)*T/6;
vnenz(k+1)=vnenz(k)+(M31+2*M32+2*M33+M34)*T/6;

%位置的解算,0.02秒更新一次
N11=vnenx(k)/(Re+h(k))/cos(lat(k));
N21=vneny(k)/(Re+h(k));
N31=vnenz(k);
N12=(vnenx(k)+vnenx(k+1))/2/(Re+h(k)+T*N31/2)/cos(lat(k)+T*N21/2);
N22=(vneny(k)+vneny(k+1))/2/(Re+h(k)+T*N31/2);
N32=(vnenz(k)+vnenz(k+1))/2;
N13=(vnenx(k)+vnenx(k+1))/2/(Re+h(k)+T*N32/2)/cos(lat(k)+T*N22/2);
N23=(vneny(k)+vneny(k+1))/2/(Re+h(k)+T*N32/2);
N33=(vnenz(k)+vnenz(k+1))/2;
N14=vnenx(k+1)/(Re+h(k)+T*N33)/cos(lat(k)+T*N23);
N24=vneny(k+1)/(Re+h(k)+T*N33);
N34=vnenz(k+1);
long(k+1)=long(k)+(N11+2*N12+2*N13+N14)*T/6;
lat(k+1)=lat(k)+(N21+2*N22+2*N23+N24)*T/6;
h(k+1)=h(k)+(N31+2*N32+2*N33+N34)*T/6;

end;
data2=xlsread('GPS.xlsx');
data2=data2';
vnenx1=data2(4,:);vnenx1=vnenx1(1:2:num*2);vneny1=data2(5,:);vneny1=vneny1(1:2:num*2);vnenz1=data2(6,:);vnenz1=vnenz1(1:2:num*2);
% data3=xlsread('GPS.xlsx');
% data3=data3';
lat1=data2(2,:);lat1=lat1(1:2:num*2);long1=data2(1,:);long1=long1(1:2:num*2);h1=data2(3,:);h1=h1(1:2:num*2);
figure(1);
% subplot(3,3,1);
% plot(t,f*180/pi,t,f1*180/pi,'red');title('俯仰角');
% subplot(3,3,2);
% plot(t,g*180/pi,t,g1*180/pi,'red');title('滚转角');
% subplot(3,3,3);
% plot(t,p*180/pi,t,p1*180/pi,'red');title('偏航角');
subplot(3,3,1);
plot(t,vnenx,t,vnenx1,'red');title('东向速度');
subplot(3,3,2);
plot(t,vneny,t,vneny1,'red');title('北向速度');
subplot(3,3,3);
plot(t,vnenz,t,vnenz1,'red');title('天向速度');
subplot(3,3,4);
plot(t,lat*180/pi,t,lat1*180/pi,'red');title('纬度');
subplot(3,3,5);
plot(t,long*180/pi,t,long1*180/pi,'red');title('经度');
subplot(3,3,6);
plot(t,h,t,h1,'red');title('高度');
% figure(2);
% subplot(3,3,1);
% plot(t,(f-f1)*180/pi);title('俯仰角误差');
% subplot(3,3,2);
% plot(t,(g-g1)*180/pi);title('滚转角误差');
% subplot(3,3,3);
% plot(t,(p-p1)*180/pi);title('偏航角误差');
% subplot(3,3,4);
% plot(t,vnenx-vnenx1);title('东向速度误差');
% subplot(3,3,5);
% plot(t,vneny-vneny1);title('北向速度误差');
% subplot(3,3,6);
% plot(t,vnenz-vnenz1);title('天向速度误差');
% subplot(3,3,7);
% plot(t,(lat-lat1)*180/pi);title('纬度误差');
% subplot(3,3,8);
% plot(t,(long-long1)*180/pi);title('经度误差');
% subplot(3,3,9);
% plot(t,h-h1);title('高度误差');
