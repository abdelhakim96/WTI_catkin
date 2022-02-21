close all
clear all
clc

set(gcf,'color','w');
%M = dlmread('inspection_1.txt');
M = dlmread('Results_costs/test1.txt');
mx = dlmread('MeshX.txt');
my = dlmread('MeshY.txt');
mz = dlmread('MeshZ.txt');


nx_160 = dlmread('nx_160.txt');
ny_160 = dlmread('ny_160.txt');
nz_160 = dlmread('nz_160.txt');

px_160 = dlmread('d10cm_interp_x');
py_160 = dlmread('d10cm_interp_y');
pz_160 = dlmread('d10cm_interp_z');


path=dlmread('path_half.txt');

xq=[0:0.032:160]
%px_5000_o = interp1([1:length(px_160)],px_160,xq);
%py_5000_o = interp1([1:length(px_160)],py_160,xq);
%pz_5000_o = interp1([1:length(px_160)],pz_160,xq);


px_5000 = dlmread('px_fit_interp_5000.txt');
py_5000 = dlmread('py_fit_interp_5000.txt');
pz_5000 = dlmread('pz_fit_interp_5000.txt');


px_5000_2 = dlmread('px_fit_interp_2_5000.txt');
py_5000_2 = dlmread('py_fit_interp_2_5000.txt');
pz_5000_2 = dlmread('pz_fit_interp_2_5000.txt');






px = dlmread('px_s.txt');
py = dlmread('py_s.txt');
pz = dlmread('pz_s.txt');

%C = dlmread('coverage.txt');
%C1 = dlmread('hakim.txt');
ts=1;
grid on
%yawset(gcf,'color','w');
figure
M=M(ts:end,:);
x=M(:,5:7);
p=M(:,8:10);
n=M(:,11:13);
kkt=M(:,26);
obj=M(:,27);
vel_d=M(:,14:16);
angles_d=M(:,17:19);
rates_d=M(:,28:30);
x_ref=M(:,2:4);
a= p-x;
a_ref=p-x_ref;
yaw=M(:,19);
yaw=yaw*pi/180;
for i=1:length(a)
s1(i,1)=(cos(yaw(i))*a(i,1)+sin(yaw(i))*a(i,2))/norm(a(i,1:2));
s2(i,1)=(a(i,1)^2+a(i,2)^2)^0.5;
s22(i,1)=norm(a_ref(i,1:2));
s3(i,1)=a(i,1)*n(i,1)+a(i,1)*n(i,2);
s4(i,1)=a(i,1)*n(i,2)-a(i,2)*n(i,1);
end

s3=a(:,1).*n(:,1)+a(:,2).*n(:,2);

ez=abs(x(:,3)-x_ref(:,3));

t=M(:,1)-M(ts,1);
%plot3(M(1321:26954,8),M(1321:26954,9),M(1321:26954,10),'k', 'LineWidth', 3.0,'LineSmoothing', 'on');
set(gcf,'color','w');
grid minor
%xlim([0 510])
plot3(x(:,1),x(:,2),x(:,3),'k', 'LineWidth', 3.0,'LineSmoothing', 'on');
hold on
plot3(x_ref(:,1),x_ref(:,2),x_ref(:,3),'g', 'LineWidth', 3.0,'LineSmoothing', 'on');
hold on
patch(mx'+68,my'-32,mz'-70,'r','EdgeColor','k','FaceAlpha',0.7);
set(gcf,'color','w');

figure 

set(gcf,'color','w');

plot(t,s1, 'b', 'LineWidth', 3.0,'LineSmoothing', 'on');

xlabel('time [s]');
ylabel('s1 objective []');
xlim([0 3000])
%legend('s1 objective')
%% Distance Plot
figure
set(gcf,'color','w');
%plot(t,s22, 'k','LineWidth', 3.0,'LineSmoothing', 'on');
hold on
plot(t,s2,'b','LineWidth', 3.0,'LineSmoothing', 'on');
hold on
yline(10,'--','LineWidth', 3.0,'Color',[0 0 0.1]);

yline(11.5,'--','LineWidth', 3.0);

xlim([50 1100])

text(490,10.08,'safe distance')
text(500,11.43,'maximum distance')


%legend('MPC obtained trajectory', 'Minimum Safe Distance', 'Max distance')
xlabel('time [s]');
ylabel('Distance from turbine [m]');
figure 

plot(t(1:20:end),s1(1:20:end)*100, 'k', 'LineWidth', 3.0,'LineSmoothing', 'on');
hold on
xlabel('time [s]');
ylabel('Coverage %');
count=0;
for i=1:length(p)-1
   
 if ((p(i,1)-p(i+1,1)) ~= 0 ||  (p(i,2)-p(i+1,2)) ~= 0 || (p(i,3)-p(i+1,3)) ~= 0) && count> 100
     event(i)=1;
     count=0;
 else
     event(i)=0;
     count=count+1;
 end
     
 
 
     
end
event(i+1)=0;
figure

set(gcf,'color','w');

plot3(x_ref(:,1),x_ref(:,2),x_ref(:,3),'g', 'LineWidth', 3.0,'LineSmoothing', 'on');
hold on
scatter3(x_ref(:,1),x_ref(:,2),x_ref(:,3),10,s22, 'filled');

figure

set(gcf,'color','w');

plot3(x_ref(:,1)-68,x_ref(:,2)+32,x_ref(:,3)+70,'k', 'LineWidth', 3.0,'LineSmoothing', 'on');
hold on
scatter3(x(:,1)-68,x(:,2)+32,x(:,3)+70,10,s2, 'filled');
hold on
patch(mx',my',mz','w','EdgeColor','k','FaceAlpha',0.7);
xlabel('x [m]');
ylabel('y [m]');
zlabel('z [m]');
legend('Waypoint trajectory','MPC obtained trajectory')
cb1 = colorbar(); 
ylabel(cb1, 'distance from turbine [m]')


%scatter(t(1:50:end), s1(1:50:end))
figure 

plot(t(1:20:end),-s3(1:20:end), 'r', 'LineWidth', 3.0,'LineSmoothing', 'on');
hold on
plot(t(1:20:end),s2(1:20:end), 'b', 'LineWidth', 3.0,'LineSmoothing', 'on');

xlabel('time [s]');
ylabel('s3');

figure
set(gcf,'color','w');

%xlim([0 510])

plot3(p(:,1),p(:,2),p(:,3),'b', 'LineWidth', 3.0,'LineSmoothing', 'on');
hold on
%plot3(x_ref(:,1),x_ref(:,2),x_ref(:,3),'w', 'LineWidth', 3.0,'LineSmoothing', 'on');
hold on
patch(mx'+68,my'-32,mz'-70,'k','EdgeColor','w','FaceAlpha',0.7);
pbaspect([1 12 12])
axis off
set(gcf,'color','w');
figure
plot (t,p(:,1),'r')
hold on
plot (t,p(:,2),'b')
hold on	
plot (t,p(:,3),'k')
hold on

for i =2:length(p)
vx(i)=(p(i,1)-p(i-1,1))/0.01;
vy(i)=(p(i,2)-p(i-1,2))/0.01;
vz(i)=(p(i,3)-p(i-1,3))/0.01;

end

figure
plot (t,vx,'r')
hold on
plot (t,vy,'b')
hold on	
plot (t,vz,'k')
hold on

figure
plot (t,x(:,1),'k','LineWidth', 2.0)
hold on
plot (t,x(:,2),'b','LineWidth', 2.0)
%hold on	
%plot (t,x(:,3),'k')

hold on
plot (t,p(:,1),'r')
hold on
plot (t,p(:,2),'g')

figure 
plot (t, s2)
xlim([30 1200])

%% Plotting the 2-D Trajectory 
figure 
set(gcf,'color','w');
xlim([50 1100])
hold on
plot (t,p(:,1),'--','Color',[0 0 0],'LineWidth', 2.0)
hold on

plot (t,x(:,1),'-','Color',[0 0 0],'LineWidth', 2.0)

xlabel('time [s]') 
ylabel('distance [m]') 

hold on
xlim([50 1100])
legend('p_x','x')

figure
set(gcf,'color','w');

plot (t,p(:,2),'--','Color',[1 0 0],'LineWidth', 2.0)
hold on

plot (t,x(:,2),'-','Color',[1 0 0],'LineWidth', 2.0)
hold on
xlim([50 1100])
legend('p_y','y')

xlabel('time [s]') 
ylabel('distance [m]') 
figure
set(gcf,'color','w');
xlim([50 1100])
plot (t,p(:,3),'--','Color',[0 0 1],'LineWidth', 2.0)
hold on
xlim([50 1100])
plot (t,x(:,3),'-','Color',[0 0 1],'LineWidth', 2.0)



xlabel('time [s]') 
ylabel('distance [m]') 


legend('p_z','z')



%% Plotting costs 

figure
set(gcf,'color','w');

xlim([50 1100])
C_h= 80*(s1-1).^2;
C_d= 200*(s2-10).^2;
C_r=  60*(s3+12.5).^2;



plot (t,C_h,'-','Color',[0 0 1],'LineWidth', 2.0)
hold on
plot (t,C_d,'-','Color',[0 1 0],'LineWidth', 2.0)
hold on
plot (t,C_r,'-','Color',[1 0 0],'LineWidth', 2.0)
xlim([50 1100])
xlabel('time [s]') 
ylabel('Cost') 
legend('C_h','C_d', 'C_r')


figure 
set(gcf,'color','w');
plot (t,s4,'-','Color',[0 0 1],'LineWidth', 2.0)
xlim([50 1100])
xlabel('time [s]') 
ylabel('s4') 




figure 
set(gcf,'color','w');
plot (t,kkt,'-','Color',[0 0 1],'LineWidth', 2.0)
xlim([50 1100])
xlabel('time [s]') 
ylabel('KKT') 

figure
set(gcf,'color','w');
plot (t,obj,'-','Color',[0 0 1],'LineWidth', 2.0)
xlim([50 1100])
xlabel('time [s]') 
ylabel('Objective') 


 figure
set(gcf,'color','w');
plot (t,vel_d(:,1),'-','Color',[0 0 1],'LineWidth', 2.0)

hold on
plot (t,vel_d(:,2),'-','Color',[1 0 0],'LineWidth', 2.0)

hold on
plot (t,vel_d(:,3),'-','Color',[0 1 0],'LineWidth', 2.0)

vel_abs=(vel_d(:,1).^2+vel_d(:,2).^2+vel_d(:,3).^2).^0.5;




xlim([50 1100])
xlabel('time [s]') 
ylabel('velocity') 

legend('vx','vy', 'vz')

figure 
%% Plot Velocities

figure 
hold on
plot (t,vel_abs,'-','Color',[0 0 1],'LineWidth', 2.0)
xlabel('time [s]')
ylabel('velocity [m/s]')

xlim ([20 1050])
%% Plot point trajectory
figure
plot3(px,py,pz,'b', 'LineWidth', 3.0,'LineSmoothing', 'on');
xlabel('x')
ylabel('y')
zlabel('z')


%% Plot interpolatted
figure
plot3(px_5000,py_5000,pz_5000,'k', 'LineWidth', 3.0);
hold on 
plot3(px_160,py_160,pz_160,'b', 'LineWidth', 3.0);
pbaspect([1 10 10])

xlabel('x')
ylabel('y')
zlabel('z')

%% plot 2-D interpolated

figure 
hold on
plot (linspace(0,1,length(px_5000)),px_5000,'-','Color',[0 0 1],'LineWidth', 2.0)
xlabel('time [s]')
ylabel('x [m]')

hold on 


%plot ([1:length(px)],px,'--','Color',[0 0 0],'LineWidth', 2.0)



hold on 


plot (linspace(0,1,length(px_160)),px_160,'--','Color',[1 0 0],'LineWidth', 2.0)


hold on

plot (linspace(0,1,length(px_5000_2)),px_5000_2,'-','Color',[0 0.5 0.5],'LineWidth', 2.0)
xlabel('time [s]')
ylabel('x [m]')








figure 
hold on
plot ([1:length(py_5000)],py_5000,'-','Color',[0 0 1],'LineWidth', 2.0)

hold on 


plot ([1:length(py)],py,'--','Color',[0 0 0],'LineWidth', 2.0)



hold on 


plot ([1:length(py_160)],py_160,'--','Color',[1 0 0],'LineWidth', 2.0)


hold on

plot ([1:length(py_5000_2)],py_5000_2,'-','Color',[0 0.5 0.5],'LineWidth', 2.0)
xlabel('time [s]')
ylabel('y [m]')


figure 
hold on
plot ([1:length(pz_5000)],pz_5000,'-','Color',[1 0 0],'LineWidth', 2.0)
xlabel('time [s]')
ylabel('z [m]')



px_inter = interp1(1:length(px_160), px_160, linspace(1, length(px_160), 10000), 'nearest');
py_inter = interp1(1:length(py_160), py_160, linspace(1, length(py_160), 10000), 'nearest');
pz_inter = interp1(1:length(pz_160), pz_160, linspace(1, length(pz_160), 10000), 'nearest');
nx_inter = interp1(1:length(nx_160), nx_160, linspace(1, length(nx_160), 10000), 'nearest');
ny_inter = interp1(1:length(ny_160), ny_160, linspace(1, length(ny_160), 10000), 'nearest');
nz_inter = interp1(1:length(nz_160), nz_160, linspace(1, length(nz_160), 10000), 'nearest');


for i=1:length(path)/2
path([i],:) = [];
end


wpx_inter = interp1(1:length(path(:,1)), path(:,1), linspace(1, length(path(:,1)), 10000), 'nearest');
wpy_inter = interp1(1:length(path(:,2)), path(:,2), linspace(1, length(path(:,2)), 10000), 'nearest');
wpz_inter = interp1(1:length(path(:,3)), path(:,3), linspace(1, length(path(:,3)), 10000), 'nearest');

wp_inter(:,1)=smooth(wpx_inter,150);
wp_inter(:,2)=smooth(wpy_inter,150);
wp_inter(:,3)=smooth(wpz_inter,150);


nx_inter=smooth(nx_inter,100);
ny_inter=smooth(ny_inter,100);
nz_inter=smooth(nz_inter,100);


for i=2:length(px_inter)
    
    vx_inter(i)=(px_inter(i)-px_inter(i-1));
    vy_inter(i)=(py_inter(i)-py_inter(i-1));
    vz_inter(i)=(pz_inter(i)-pz_inter(i-1));
    if vx_inter(i)<0.01
        vx_inter(i)=vx_inter(i-1);
    end
    if vy_inter(i)<0.01
        vy_inter(i)=vy_inter(i-1);
    end
    if vz_inter(i)<0.01
        vz_inter(i)=vz_inter(i-1);
    end
    
    wp(i,1:3)=0;

end


vx_inter=smooth(vx_inter,100);
vy_inter=smooth(vy_inter,100);
vz_inter=smooth(vz_inter,100);


plot ([1:length(wpx_inter)],wp_inter(:,1),'-','Color',[1 0 0],'LineWidth', 2.0)
hold on
plot ([1:length(wpy_inter)],wp_inter(:,2),'-','Color',[0 0 1],'LineWidth', 2.0)
hold on
plot ([1:length(wpz_inter)],wp_inter(:,3),'-','Color',[0 1 0],'LineWidth', 2.0)




writematrix(px_inter', "px_inter.txt");

writematrix(py_inter', "py_inter.txt");

writematrix(pz_inter', "pz_inter.txt");

writematrix(nx_inter, "nx_inter.txt");

writematrix(ny_inter, "ny_inter.txt");

writematrix(nz_inter, "nz_inter.txt");

writematrix(vx_inter, "vx_inter.txt");

writematrix(vy_inter, "vy_inter.txt");

writematrix(vz_inter, "vz_inter.txt");


%writematrix(wp, "wp.txt");
%ritematrix(wp_inter, "wp_inter.txt");




plot ([1:length(wpx_inter)],wpx_inter,'-','Color',[0 1 0],'LineWidth', 2.0)
