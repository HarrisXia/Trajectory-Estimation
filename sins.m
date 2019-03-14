clear;
%经度 l=120*pi/180;经度 al=45*pi/180;高度 h=100;初始速度 v0=[0;200;0];
%子午圈曲率半径 rn=re*(1-2*e+3*e*sin(al)^2);卯酉圈曲率半径 rm=re*(1+e*sin(al)^2);
re=6378393;
e=1/298.3;
%飞行轨迹仿真 
tf=1*200;
delt=0.02;
for t=1:tf
    l(1)=120*pi/180;
    al(1)=45*pi/180;
    h(1)=100;
    v(:,1)=[0;200;0];
    y(1)=0;
    y0(t)=pi/200;
    y(t+1)=y(t)+y0(t)*delt;
    at(:,t)=[0;0;y0(t)*v(2,t)];
    a(:,t)=[1,0,0;0,cos(y(t)),-sin(y(t));0,sin(y(t)),cos(y(t))]*at(:,t);
    v(1,t+1)=v(1,t)+a(1,t)*delt;
    v(2,t+1)=v(2,t)+a(2,t)*delt;
    v(3,t+1)=v(3,t)+a(3,t)*delt;
    rn(t+1)=re*(1-2*e+3*e*sin(al(t))^2);
    rm(t+1)=re*(1+e*sin(al(t))^2);
    al(t+1)= al(t)+v(2,t+1)./(rn(t+1)+h(t))*delt;
    l(t+1)= l(t)+v(1,t+1)./(rm(t+1)+h(t))./cos(al(t))*delt;
    h(t+1)=h(t)+v(3,t+1)*delt;
end
for t=tf+1:tf*2
    y0(t)=0;
    y(t)=y(tf);
    at(:,t)=[0;0;0];
    a(:,t)=at(:,t);
    v(1,t+1)=v(1,t)+a(1,t)*delt;
    v(2,t+1)=v(2,t)+a(2,t)*delt;
    v(3,t+1)=v(3,t)+a(3,t)*delt;
    rn(t+1)=re*(1-2*e+3*e*sin(al(t))^2);
    rm(t+1)=re*(1+e*sin(al(t))^2);
    al(t+1)= al(t)+v(2,t+1)./(rn(t+1)+h(t))*delt;
    l(t+1)= l(t)+v(1,t+1)./(rm(t+1)+h(t))./cos(al(t))*delt;
    h(t+1)=h(t)+v(3,t+1)*delt;
end
for t=tf*2:tf*3
    y0(t)=-pi/200;
    y(t+1)=y(t)+y0(t)*delt;
    at(:,t)=[0;0;y0(t)*v(2,t)];
    a(:,t)=[1,0,0;0,cos(y(t)),-sin(y(t));0,sin(y(t)),cos(y(t))]*at(:,t);
    v(1,t+1)=v(1,t)+a(1,t)*delt;
    v(2,t+1)=v(2,t)+a(2,t)*delt;
    v(3,t+1)=v(3,t)+a(3,t)*delt;
    rn(t+1)=re*(1-2*e+3*e*sin(al(t))^2);
    rm(t+1)=re*(1+e*sin(al(t))^2);
    al(t+1)= al(t)+v(2,t+1)./(rn(t+1)+h(t))*delt;
    l(t+1)= l(t)+v(1,t+1)./(rm(t+1)+h(t))./cos(al(t))*delt;
    h(t+1)=h(t)+v(3,t+1)*delt;
end
for t=tf*3+1:tf*9
    at(:,t)=[0;0;0];
    a(:,t)=at(:,t);
    v(:,t)=v(:,tf*3+1);
    v(1,t+1)=v(1,t)+a(1,t)*delt;
    v(2,t+1)=v(2,t)+a(2,t)*delt;
    v(3,t+1)=v(3,t)+a(3,t)*delt;
    y0(t)=0;
    y(t)=y(tf*3+1);
    rn(t+1)=re*(1-2*e+3*e*sin(al(t))^2);
    rm(t+1)=re*(1+e*sin(al(t))^2);
    al(t+1)= al(t)+v(2,t+1)./(rn(t+1)+h(t))*delt;
    l(t+1)= l(t)+v(1,t+1)./(rm(t+1)+h(t))./cos(al(t))*delt;
    h(t+1)=h(t)+v(3,t)*delt;
end
for t=tf*9:tf*10
    y0(t)=-pi/200;
    y(t+1)=y(t)+y0(t)*delt;
    at(:,t)=[0;0;y0(t)*v(2,t)];
    a(:,t)=[1,0,0;0,cos(y(t)),-sin(y(t));0,sin(y(t)),cos(y(t))]*at(:,t);
    v(1,t+1)=v(1,t)+a(1,t)*delt;
    v(2,t+1)=v(2,t)+a(2,t)*delt;
    v(3,t+1)=v(3,t)+a(3,t)*delt;
    rn(t+1)=re*(1-2*e+3*e*sin(al(t))^2);
    rm(t+1)=re*(1+e*sin(al(t))^2);
    al(t+1)= al(t)+v(2,t+1)./(rn(t+1)+h(t))*delt;
    l(t+1)= l(t)+v(1,t+1)./(rm(t+1)+h(t))./cos(al(t))*delt;
    h(t+1)=h(t)+v(3,t+1)*delt;
end
for t=tf*10+1:tf*11
    y0(t)=0;
    y(t)=y(tf*10+1);
    at(:,t)=[0;0;0];
    a(:,t)=at(:,t);
    v(1,t+1)=v(1,t)+a(1,t)*delt;
    v(2,t+1)=v(2,t)+a(2,t)*delt;
    v(3,t+1)=v(3,t)+a(3,t)*delt;
    rn(t+1)=re*(1-2*e+3*e*sin(al(t))^2);
    rm(t+1)=re*(1+e*sin(al(t))^2);
    al(t+1)= al(t)+v(2,t+1)./(rn(t+1)+h(t))*delt;
    l(t+1)= l(t)+v(1,t+1)./(rm(t+1)+h(t))./cos(al(t))*delt;
    h(t+1)=h(t)+v(3,t+1)*delt;
end
for t=tf*11:tf*12
    y0(t)=pi/200;
    y(t+1)=y(t)+y0(t)*delt;
    at(:,t)=[0;0;y0(t)*v(2,t)];
    a(:,t)=[1,0,0;0,cos(y(t)),-sin(y(t));0,sin(y(t)),cos(y(t))]*at(:,t);
    v(1,t+1)=v(1,t)+a(1,t)*delt;
    v(2,t+1)=v(2,t)+a(2,t)*delt;
    v(3,t+1)=v(3,t)+a(3,t)*delt;
    rn(t+1)=re*(1-2*e+3*e*sin(al(t))^2);
    rm(t+1)=re*(1+e*sin(al(t))^2);
   al(t+1)= al(t)+v(2,t+1)./(rn(t+1)+h(t))*delt;
    l(t+1)= l(t)+v(1,t+1)./(rm(t+1)+h(t))./cos(al(t))*delt;
    h(t+1)=h(t)+v(3,t+1)*delt;
end

%加速度计输出，陀螺输出,及捷联惯导系统输出
w=15.04107*pi/180/3600;%地球自转角速率
vv(:,1)=v(:,1);
l0(1)=120*pi/180;
a0(1)=45*pi/180;
h0(1)=100;

for t= 1:tf*12
g(t)=9.7803+0.051799*sin(al(t))^2-(0.94114e-10)*h(t);
rn(t)=re*(1-2*e+3*e*sin(al(t))^2);
rm(t)=re*(1+e*sin(al(t))^2); 
wie(1,t)=0;
wie(2,t)=w*cos(al(t));
wie(3,t)=w*sin(al(t));
%wie=[wie(1,t);wie(2,t);wie(3,t)];
wen(1,t)=-v(2,t)./(rn(t)+h(t));
wen(2,t)=v(1,t)./(rm(t)+h(t));
wen(3,t)=v(1,t)./(rm(t)+h(t)).*tan(al(t));
ww=(2*wie+wen);
ctn(:,:,t)=[1,0,0;0,cos(y(t)),-sin(y(t));0,sin(y(t)),cos(y(t))];
cnb(:,:,t)=ctn(:,:,t)';
fn(:,t)=ctn(:,:,t)*at(:,t)+[0,-ww(3,t),ww(2,t);ww(3,t),0,-ww(1,t);-ww(2,t),ww(1,t),0]*v(:,t)+[0;0;g(t)];
fb(:,t)=cnb(:,:,t)*fn(:,t);
wib(:,t)=cnb(:,:,t)*([y0(t);0;0]+wen(:,t)+wie(:,t));

%捷联惯导系统
%姿态矩阵初值
T(:,:,1)=ctn(:,:,1);
%T(:,:,1)=inv([0,0,-g(1);0,w*cos(pi/4),w*sin(pi/4);g(1)*w*cos(pi/4),0,0])*[fb(1,1),fb(2,1),fb(3,1);wib(1,1),wib(2,1),wib(3,1);
%    wib(3,1)*fb(2,1)-wib(2,1)*fb(3,1),wib(1,1)*fb(3,1)-wib(3,1)*fb(1,1),wib(2,1)*fb(1,1)-wib(1,1)*fb(2,1)];
%T(:,:,1)=[0,0,sec(a0(1))/g(1)/w;tan(a0(1))/g(1),sec(a0(1))/w,0;-1/g(1),0,0]*[fb(1,1),fb(2,1),fb(3,1);wib(1,1),wib(2,1),wib(3,1);
%    wib(3,1)*fb(2,1)-wib(2,1)*fb(3,1),wib(1,1)*fb(3,1)-wib(3,1)*fb(1,1),wib(2,1)*fb(1,1)-wib(1,1)*fb(2,1)];

%T(:,:,1)=[sec(a0(1))/g(1)/w*(wib(3,1)*fb(2,1)-wib(2,1)*fb(3,1)),sec(a0(1))/g(1)/w*(wib(1,1)*fb(3,1)-wib(3,1)*fb(1,1)),sec(a0(1))/g(1)/w*(wib(2,1)*fb(1,1)-wib(1,1)*fb(2,1));
%    fb(1,1)/g(1)*tan(a0(1))+wib(1,1)/w*sec(a0(1)),fb(2,1)/g(1)*tan(a0(1))+wib(2,1)/w*sec(a0(1)),fb(3,1)/g(1)*tan(a0(1))+wib(3,1)/w*sec(a0(1));
%    -fb(1,1)/g(1),-fb(2,1)/g(1),-fb(3,1)/g(1)];
fn(:,t)=T(:,:,t)*fb(:,t);
dvv(1,t)=fn(1,t)+(2*w*sin(a0(t))+vv(1,t).*tan(a0(t))./(rm(t)+h(t))).*vv(2,t)-(2*w*cos(a0(t))+vv(1,t)./(rm(t)+h(t))).*vv(3,t);
dvv(2,t)=fn(2,t)-(2*w*sin(a0(t))+vv(1,t).*tan(a0(t))./(rm(t)+h(t))).*vv(1,t)-vv(2,t).*vv(3,t)./(rn(t)+h(t));
dvv(3,t)=fn(3,t)+(2*w*cos(a0(t))+vv(1,t)./(rm(t)+h(t))).*vv(1,t)+vv(2,t).*vv(2,t)./(rn(t)+h(t))-g(t);
vv(1,t+1)=vv(1,t)+dvv(1,t)*delt;
vv(2,t+1)=vv(2,t)+dvv(2,t)*delt;
vv(3,t+1)=vv(3,t)+dvv(3,t)*delt;
wep(:,t)=[-vv(2,t)/(rn(t)+h(t));vv(1,t)/(rm(t)+h(t));vv(1,t)*tan(a0(t))/(rm(t)+h(t))];
wpb(:,t)=wib(:,t)-inv(T(:,:,t))*(wep(:,t)+wie(:,t));
%位置矩阵
c(:,:,1)=[-sin(l0(1)),cos(l0(1)),0;-sin(a0(1))*cos(l0(1)),-sin(a0(1))*sin(l0(1)),cos(a0(1));cos(a0(1))*cos(l0(1)),cos(a0(1))*sin(l0(1)),sin(a0(1))];
dc(:,:,t)=[0,wep(3,t),-wep(2,t);-wep(3,t),0,wep(1,t);wep(2,t),-wep(1,t),0]*c(:,:,t);
c(:,:,t+1)=c(:,:,t)+dc(:,:,t)*delt;
a0(t+1)=asin(c(3,3,t+1));
if c(3,1,t+1)>0
l0(t+1)=atan(c(3,2,t+1)/c(3,1,t+1));
elseif c(3,2,t+1)>0
   l0(t+1)=atan(c(3,2,t+1)/c(3,1,t+1))+pi; 
else
   l0(t+1)=atan(c(3,2,t+1)/c(3,1,t+1))-pi; 
end
h0(t+1)=h0(t)+vv(3,t+1)*delt;

%四元数计算
if T(3,2,1)>T(2,3,1)
    q1(1)=0.5*sqrt(1+T(1,1,1)-T(2,2,1)-T(3,3,1));
else
    q1(1)=-(0.5*sqrt(1+T(1,1,1)-T(2,2,1)-T(3,3,1)));
end
if T(1,3,1)>T(3,1,1)
    q2(1)=0.5*sqrt(1-T(1,1,1)+T(2,2,1)-T(3,3,1)); 
else
    q2(1)=-(0.5*sqrt(1-T(1,1,1)+T(2,2,1)-T(3,3,1)));
end
if T(2,1,1)>T(1,2,1)
    q3(1)=0.5*sqrt(1-T(1,1,1)-T(2,2,1)+T(3,3,1)); 
else
    q3(1)=-(0.5*sqrt(1-T(1,1,1)-T(2,2,1)+T(3,3,1)));
end
q0(1)=sqrt(1-q1(1)^2-q2(1)^2-q3(1)^2);
Q(:,t)=[q0(t);q1(t);q2(t);q3(t)];
dQ(:,t)=0.5*[0,-wpb(1,t),-wpb(2,t),-wpb(3,t);
    wpb(1,t),0,wpb(3,t),-wpb(2,t);
    wpb(2,t),-wpb(3,t),0,wpb(1,t);
    wpb(3,t),wpb(2,t),-wpb(1,t),0]*Q(:,t);
Q(:,t+1)=Q(:,t)+dQ(:,t)*delt;
%四元数的归一化
q0(t+1)=Q(1,t+1)/(sqrt(Q(1,t+1)^2+Q(2,t+1)^2+Q(3,t+1)^2+Q(4,t+1)^2));
q1(t+1)=Q(2,t+1)/(sqrt(Q(1,t+1)^2+Q(2,t+1)^2+Q(3,t+1)^2+Q(4,t+1)^2));
q2(t+1)=Q(3,t+1)/(sqrt(Q(1,t+1)^2+Q(2,t+1)^2+Q(3,t+1)^2+Q(4,t+1)^2));
q3(t+1)=Q(4,t+1)/(sqrt(Q(1,t+1)^2+Q(2,t+1)^2+Q(3,t+1)^2+Q(4,t+1)^2));
T(:,:,t+1)=[q0(t+1)^2+q1(t+1)^2-q2(t+1)^2-q3(t+1)^2,2*(q1(t+1)*q2(t+1)-q0(t+1)*q3(t+1)),2*(q1(t+1)*q3(t+1)+q0(t+1)*q2(t+1));
    2*(q1(t+1)*q2(t+1)+q0(t+1)*q3(t+1)),q0(t+1)^2-q1(t+1)^2+q2(t+1)^2-q3(t+1)^2,2*(q2(t+1)*q3(t+1)-q0(t+1)*q1(t+1));
    2*(q1(t+1)*q3(t+1)-q0(t+1)*q2(t+1)),2*(q2(t+1)*q3(t+1)+q0(t+1)*q1(t+1)),q0(t+1)^2-q1(t+1)^2-q2(t+1)^2+q3(t+1)^2];
end

i=1:tf*12;
%subplot(311) ,plot(al(:)/pi*180),hold on;plot(a0(:)/pi*180,'--red');title('纬度/度');grid on;
%subplot(312),plot(l(:)/pi*180),hold on;plot(l0(:)/pi*180,'--red');title('经度/度');grid on;
%subplot(313),plot(h(:)),hold on;plot(h0(:),'--red');title('经度/米');grid on;
 figure;
 plot3(l(:)/pi*180,al(:)/pi*180,h(:),'b:');grid on;hold on;%xlabel('经度'),ylabel('纬度'),zlabel('高度');
 plot3(l0(:)/pi*180,a0(:)/pi*180,h(:),'m:'),grid on;xlim([119.9996 120.0004]);xlabel('经度/度'),ylabel('纬度/度'),zlabel('高度/米');title('捷联系统跟踪轨迹')
 
 %set(xlabel('纬度')，ylabel('经度'),zlabel('高度'));
 %plot3(al(i)/pi*180,l(i)/pi*180,h(i));grid on;hold on;
 %plot3(a0(i)/pi*180,l0(i)/pi*180,h(i),'color','red');grid on,axis auto;
 %plot3(a0(i)-al(i),l0(i)-l(i),h(i),'color','black');grid on;hold on;
 %figure
%subplot(312) ,plot(v(2,:)),hold on;plot(vv(2,:),'--red');title('北向速度');grid on;
%subplot(311),plot(v(1,:)),hold on;plot(vv(1,:),'--red');title('东向速度');grid on;
%subplot(313),plot(v(3,:)),hold on;plot(vv(3,:),'--red');title('垂直速度');grid on;
figure;
subplot(311) ,plot((al(:)-a0(:))*re);title('纬度误差/m');grid on;
subplot(312),plot((l(:)-l0(:))*re);title('经度误差/m');grid on;
subplot(313),plot((h(:)-h0(:)));title('高度误差/m');grid on;
