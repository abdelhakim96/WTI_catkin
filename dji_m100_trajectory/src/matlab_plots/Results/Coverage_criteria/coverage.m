close all
clc
clear all

%input data
%M = dlmread('/home/hakim/catkin_ws/src/WTI_catkin/dji_m100_trajectory/src/matlab_plots/Results/Coverage_criteria/Results_2/test2.txt');
M = dlmread('/home/hakim/catkin_ws/src/WTI_catkin/dji_m100_trajectory/src/matlab_plots/Results/Results_vel_1/test1.txt');
mx = dlmread('mesh_x.txt');
my = dlmread('mesh_y.txt');
mz = dlmread('mesh_z.txt');
px =dlmread('px_160.txt');
py =dlmread('py_160.txt');
pz =dlmread('pz_160.txt');
nx =dlmread('nx_160.txt');
ny =dlmread('ny_160.txt');
nz =dlmread('nz_160.txt');

mx=mx';
my=my';
mz=mz';
psi=M(:,19);
x=M(:,5:7);
x_ref=M(:,2:4);

%px=mx(:,1)+mx(:,1)+mx(:,1);

px=px;
py=py;
pz=pz;




%s=a(:,1).*n(:,1)+a(:,2).*n(:,2);





x(:,1)=x(:,1)-68; x(:,2)=x(:,2)+32;
x(:,3)=x(:,3)+70;
%camera

d=10000;


%ax=px-nx;
%ay=py-ny;
hh=0;%
for j=1:length(px)
for i=1:length(x)

    
    %d1= ((x(i,1)-px(j))^2+(x(i,2)-py(j))^2+(x(i,3)-pz(j))^2)^0.5;
    d1= ((x(i,1)-px(j))^2+(x(i,2)-py(j))^2)^0.5;
    
    
    %d1= ((x(i,1)-px(j))^2+(x(i,2)-py(j))^2)^0.5;
    s3i=((px(j)-x(i,1))*nx(j)+(py(j)-x(i,2))*ny(j));
    s4=((px(j)-x(i,1))*ny(j)-(py(j)-x(i,2))*nx(j));

    if ( ((d1-10.5)^2<(d-10.5)^2) && ((px(j)-x(i,1))*nx(j)+(py(j)-x(i,2))*ny(j))<-6 && d1>2  && (x(i,3)-pz(j))^2<1)   
        %&& ((x(i,3)-pz(j))^2)^0.5<1
        d=d1;
        n=i;  
        hh=n;
        s3(j)=(px(j)-x(i,1))*nx(j)+(py(j)-x(i,2))*ny(j);
        s_p(j)=((px(j)-x(i,1))*ny(j)+(py(j)-x(i,2))*nx(j))/d;
        dist(j)=d;
        
        
    end  

end 

if n==0
   n=hh;
end





%ax(j)=px(j)-x(n,1);
%ay(j)=py(j)-x(n,2);
%az(j)=0;
nn(j)=n;

dd(j)=d;
x1(j,:)=x(n,:);
psi1(j,:)=psi(n,:);
x_ref1(j,:)=x_ref(n,:);
%s_p(j)=((px(j)-x(n,1))*ny(j)-(py(j)-x(n,2))*nx(j));
d=1000000;
n=0;
end





%plot3(x1(:,1),x1(:,2),x1(:,3),'k', 'LineWidth', 3.0,'LineSmoothing', 'on');





%sensor_s=8.5*0.0001*2;    %sensor size

%pixel_s=0.011;
pixel_s=0.002;
f=2000*pixel_s;
%f=20;
%focal length
res=1000;
%sensor_s=0.002*res*2;
%sensor_s=0.011;
sensor_s=8.4/10000;
pitch=-90*pi/180;
%yaw=0*pi/180;
roll=0*pi/180;

%for i=1:length(px)
count=0;
%for i=1:length(px)
for i=1:162

 p_c= [px(i);py(i);pz(i);1];
 p1=[mx(i,1);my(i,1);mz(i,1);1];
 p2=[mx(i,2);my(i,2);mz(i,2);1];
 p3=[mx(i,3);my(i,3);mz(i,3);1];
 yaw=-psi1(i)*pi/180;
 
 
 %roll=yaw;
 %yaw=0;
 %yaw=pi;
 drone_x=x1(i,:)';
% drone_x=x_ref1(i,:)';
 %yaw=pi;
 %yaw=0;
 
 
 
 
 %%trial
 
 %p_c=[10;0;0;1];
 %drone_x=[3;0;0];
 %yaw=30*pi/180;
 %pitch=-90*pi/180;
 
 %drones position

%t=[0;0;0;0];

rM=[1    0           0;
    0    cos(roll)   sin(roll);
    0    -sin(roll)  cos(roll)];
pM=[cos(pitch)      0       -sin(pitch);
    0               1       0;
    sin(pitch)      0       cos(pitch)];
yM=[cos(yaw)        sin(yaw)        0;
    -sin(yaw)       cos(yaw)        0;
    0               0               1];



%DCM=(rM*pM)*yM;
DCM=(rM*yM)*pM;
R=DCM;

t= drone_x;
%t=[0 ;0; 0];
t=[t;1];


RT(1:3,1:3)=R;


RT(1:4,4)=t;
%RT(1:4,4)=t;
RT(4,1:4)=[0 0 0 1];

%RT=inv(RT);
%RT(4,1:4)=[0 0 0 1];
%RT=inv(RT);

xx=R*drone_x;
xx=[xx;0];

%pt_c=RT*(p_c-t)-t;  %point in local coordinates
xx=[drone_x;0];

RT=inv(RT);
pt_c=RT*(p_c)

pt_1=RT*(p1);
pt_2=RT*(p2);
pt_3=RT*(p3);


u_c=f*(pt_c(2)/pt_c(3));
v_c=f*(pt_c(1)/pt_c(3));

u_1=f*(pt_1(2)/pt_1(3));
v_1=f*(pt_1(1)/pt_1(3));

u_2=f*(pt_2(2)/pt_2(3));
v_2=f*(pt_2(1)/pt_2(3));


u_3=f*(pt_3(2)/pt_3(3));
v_3=f*(pt_3(1)/pt_3(3));




if  (abs(u_c)>(sensor_s/2) || abs(v_c)>(sensor_s/2) ||   abs(u_1)>(sensor_s/2) || abs(v_1)>(sensor_s/2)) ||...   
    (abs(u_2)>(sensor_s/2) || abs(v_2)>(sensor_s/2) ||abs(u_3)>(sensor_s/2) || abs(v_3)>(sensor_s/2)) 

    out(i)=1;
    count=count+1;
else
    out(i)=0;
end


end








plot3(x(:,1),x(:,2),x(:,3),'k', 'LineWidth', 2.0,'LineSmoothing', 'on');

hold on

%scatter3(px(10:10),py(10:10),pz(10:10),10,pz(10:10), 'filled');

hold on
patch(mx',my',mz','w','EdgeColor','k','FaceAlpha',0.7);


hold on


%plot3(x1(1:40,1),x1(1:40,2),x1(1:40,3),'r', 'LineWidth', 3.0,'LineSmoothing', 'on');
set(gcf,'color','w');


for i=1:162
    if out(i)==1 
        
        scatter3(x1(i,1),x1(i,2),x1(i,3),'b','filled', 'LineWidth', 2.0,'DisplayName','hakim');
    
        hold on
        %scatter3(px(i),py(i),pz(i),10, 'k', 'filled');
        hold on
        %quiver3(x1(i,1),x1(i,2),x1(i,3),ax(i)/2,ay(i)/2,az(i),'g')
        %quiver3(nx(i,1),x1(i,2),ny(i,3),10*cos(psi1(i)*pi/180),10*sin(psi1(i)*pi/180),0,'g')
        quiver3(x1(i,1),x1(i,2),x1(i,3),4*cos(psi1(i)*pi/180),4*sin(psi1(i)*pi/180),0,'b','MaxHeadSize',1000,'LineWidth', 1.5);
 
        patch(mx(i,:),my(i,:),mz(i,:),'b');
        hold on
        
        hold on
      % patch(mx(i),my(i),mz(i),'FaceColor','r');
       % legend('path', 'turbine','hakim','aschasc')
        hold on
    else
        
         % scatter3(x1(i,1),x1(i,2),x1(i,3),'k','filled', 'LineWidth',  3.0);
         % quiver3(x1(i,1),x1(i,2),x1(i,3),4*cos(psi1(i)*pi/180),4*sin(psi1(i)*pi/180),0,'k','LineWidth', 1.0)
     
         patch(mx(i,:),my(i,:),mz(i,:),'white');
    end
       
end
%legend('hakim')
%legend('path', 'turbine')

%scatter3(x1(:,1),x1(:,2),x1(:,4),'r', 'LineWidth', 3.0);
hold on

%quiver3(px',py',pz',nx',ny',nz','g')


%%%%%%%%%%%%%

for i=1:162
    if -s3(i)<2
     s3(i)=s3(i-1);
     i
     s3(i)
     dist(i)=dist(i-1);
    end
end
    
figure
tt=1:162;
plot(tt,-s3, 'k', 'LineWidth', 3.0,'LineSmoothing', 'on');
hold on
plot(tt,dist, 'b', 'LineWidth', 3.0,'LineSmoothing', 'on');
hold on
xlabel('time [s]');
ylabel('s3');


%a1=x1(:,1)-p1;
%a2=x2(:,1)-p2;
%a3=x3(:,1)-p3;

%s4=ny*az
sin_sp=asin(s_p)*(180/pi);
figure 
plot(tt,sin_sp, 'k', 'LineWidth', 2.0,'LineSmoothing', 'on');
xlabel('time [s]');
ylabel('Incidence Angle [deg]');
