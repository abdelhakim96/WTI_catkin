close all
clc
clear all


%% plot parameters

save_dest='/home/hakim/catkin_ws/src/WTI_catkin/Results/Figures';
%turbine translation
x_shift= +68;
y_shift= -32;
z_shift= -70;





%skip values
skip=4;


%Weights
W_h=80;
W_d=200;
W_r=60;












%colors defintion
%color_VTNMPC = [0 0.4470 0.7410];
%color_PAMPC = [0 0.5 0.1];
%color_NMPC = [0.85 0.3250 0.0980];
%color_point_traj = [0.4980 0.3250 0.85];
color_point_traj = [0 0 0];
color_VTNMPC = [0 0 1];
color_PAMPC = [0 0 1];
color_NMPC = [1 0 0];




color_turbine = [0.85 0.3250 0.0980];
color_covered_turbine = [ 1 0 0];
color_Ch= [ 1 0 0];
color_Cd= [ 0 1 0];
color_Cr= [ 0 0 1];
color_Ct= [ 0 0 0];
%% Read text files


M_VT_nowind = dlmread('/home/hakim/catkin_ws/src/WTI_catkin/Results/Data/VTNMPC/sin_comb3_4ms_0.5_10hz.txt');    %VTMPC data

M_VT_wind = dlmread('/home/hakim/catkin_ws/src/WTI_catkin/Results/Data/VTNMPC/sin_comb3_4ms_0.5_10hz.txt');
%M_VT_wind = dlmread('/home/hakim/catkin_ws/src/WTI_catkin/Results/Data/VTNMPC/GT_traj.txt');


M_NMPC_nowind=dlmread('/home/hakim/catkin_ws/src/WTI_catkin/Results/Data/NMPC/sin_comb3_4ms_0.5_10hz.txt');
M_NMPC_wind=dlmread('/home/hakim/catkin_ws/src/WTI_catkin/Results/Data/NMPC/sin_comb3_4ms_0.5_10hz.txt');

M_PAMPC_nowind=dlmread('/home/hakim/catkin_ws/src/WTI_catkin/Results/Data/VTNMPC/sin_comb3_4ms_0.5_10hz.txt');
M_PAMPC_wind=dlmread('/home/hakim/catkin_ws/src/WTI_catkin/Results/Data/VTNMPC/sin_comb3_4ms_0.5_10hz.txt');



%% preprocess data

% Remove points before starting
%for i=1:length (M_VT_nowind)-2000
%    if (((M_VT_nowind(i,8)-M_VT_nowind(i+1,8)) ==0 )  && ((M_VT_nowind(i,9)-M_VT_nowind(i+1,9)) ==0 )  &&  ((M_VT_nowind(i,10)-M_VT_nowind(i+1,10)) ==0 ))
        
%       M_VT_nowind(i,:) = [];
        
        
%    end
%end
j=1;

t1_VT_nowind=100;
t2_VT_nowind=length(M_VT_nowind)-1000;

t1_VT_wind=100;
t2_VT_wind=length(M_VT_wind)-1000;

t1_NMPC_nowind=100;
t2_NMPC_nowind=length(M_NMPC_nowind)-1000;


t1_NMPC_wind=1000;
t2_NMPC_wind=length(M_NMPC_wind)-1000;


t1_PAMPC_nowind=100;
t2_PAMPC_nowind=length(M_PAMPC_wind)-1000;

t1_PAMPC_wind=100;
t2_PAMPC_wind=length(M_PAMPC_wind)-1000;







mx = dlmread('/home/hakim/catkin_ws/src/WTI_catkin/Results/Data/Mesh/MeshX.txt');
my = dlmread('/home/hakim/catkin_ws/src/WTI_catkin/Results/Data/Mesh/MeshY.txt');
mz = dlmread('/home/hakim/catkin_ws/src/WTI_catkin/Results/Data/Mesh/MeshZ.txt');




px = dlmread('/home/hakim/catkin_ws/src/WTI_catkin/Results/Data/Point_to_View_Trajectory/mp_d1cm_interp_x.txt');       %triangles mesh centres
py = dlmread('/home/hakim/catkin_ws/src/WTI_catkin/Results/Data/Point_to_View_Trajectory/mp_d1cm_interp_y.txt');
pz= dlmread('/home/hakim/catkin_ws/src/WTI_catkin/Results/Data/Point_to_View_Trajectory/mp_d1cm_interp_z.txt');

px=px+x_shift;
py=py+y_shift;
pz=pz+z_shift;

mx=mx+x_shift;
my=my+y_shift;
mz=mz+z_shift;


%{ 


t=M(:,1);                     %time
x=M(:,5:7);                   %Drone position (x,y,z)
x_pa=P(1:end,5:7);  

x(:,1)=x(:,1)+x_shift;
x(:,2)=x(:,2)+y_shift;
x(:,3)=x(:,3)+z_shift;

x_pa(:,1)=x_pa(:,1)+x_shift;
x_pa(:,2)=x_pa(:,2)+y_shift;
x_pa(:,3)=x_pa(:,3)+z_shift;



p=M(:,8:10);  %Reference point (px,py,pz)
p(:,1)=p(:,1)+x_shift;
p(:,2)=p(:,2)+y_shift;
p(:,3)=p(:,3)+z_shift;



n=M(:,11:13);   


x_gp=dlmread('plotting_data/Path_half.txt');

gp_x = interp1(1:length(x_gp(:,1)), x_gp(:,1), linspace(1, length(x_gp(:,1)), length(t)), 'nearest');
gp_y = interp1(1:length(x_gp(:,2)), x_gp(:,2), linspace(1, length(x_gp(:,2)), length(t)), 'nearest');
gp_z = interp1(1:length(x_gp(:,3)), x_gp(:,3), linspace(1, length(x_gp(:,3)), length(t)), 'nearest');



pa_x = interp1(1:length(x_pa(:,1)), x_pa(:,1), linspace(1, length(x_pa(:,1)), length(t)), 'nearest');
pa_y = interp1(1:length(x_pa(:,2)), x_pa(:,2), linspace(1, length(x_pa(:,2)), length(t)), 'nearest');
pa_z = interp1(1:length(x_pa(:,3)), x_pa(:,3), linspace(1, length(x_pa(:,3)), length(t)), 'nearest');


pa(:,1)=pa_x;
pa(:,2)=pa_y;
pa(:,3)=pa_z;



x_wp(:,1)=gp_x;
x_wp(:,2)=gp_y;
x_wp(:,3)=gp_z;
x_wp(:,1)=smooth(x_wp(:,1),150);
x_wp(:,2)=smooth(x_wp(:,1),150);
x_wp(:,3)=smooth(x_wp(:,1),150);



for i=2:length(n)-1
v_n(i)=(n(i,1)-n(i-1,1))^2+(n(i,2)-n(i-1,2))^0.5;
v_n(i)=real(v_n(i));
end


obj=M(:,26); %reference surface normal (nx,ny,nz)
kkt=M(:,27);                  %KKT error  % Total cost
vel_drone=M(:,14:16);         % drone velocity (vx,vy,vz)
angles_d=M(:,17:19);          % Angles (Roll, Pith, Yaw)
rates_d=M(:,28:30);           % Angular rates(Roll_dot, Pitch_dot, Yaw_dot)
x_ref=M(:,2:4);               % Way point trajectory %%for comparision%% (x_desired, y_desired, z_desired)
x_ref(:,1)=x_ref(:,1)+x_shift;
x_ref(:,2)=x_ref(:,2)+y_shift;
x_ref(:,3)=x_ref(:,3)+z_shift;
%x_gp(:,1)=x_gp(:,1)+x_shift;
%x_gp(:,2)=x_gp(:,2)+y_shift;
%x_gp(:,3)=x_gp(:,3)+z_shift;

a= p-x;                       % Desired heading vector a (ax,ay,az)
a_ref=p-x_ref;                % Desired heading vector a for reference trajectory (ax_d,ay_d,az_d)
yaw=M(:,19);                  % yaw angle calculation
yaw=yaw*pi/180;

%% Calculate the functions used in the Cost C_h, C_d, C_r
for i=1:length(a)
s_h(i,1)=(cos(yaw(i))*a(i,1)+sin(yaw(i))*a(i,2))/norm(a(i,1:2));    % heading function calculation s_h
s_d(i,1)=(a(i,1)^2+a(i,2)^2+a(i,3)^2)^0.5;                                   % distance function calculation s_d
s_d_wp(i,1)=norm(a_ref(i,1:2));                                     % distance function for wp trajectory s_d_wp
s_r(i,1)=a(i,1)*n(i,1)+a(i,2)*n(i,2);                               % region of inerest function
s_p(i,1)=a(i,1)*n(i,2)-a(i,2)*n(i,1);                               % perpendicularity measure
end



C_h= W_h*(s_h-1).^2;
C_d= W_d*(s_d-10).^2;
C_r=  W_r*(s_r+12.5).^2;

%}

%% Plot Trajectory 3-D 
fig_han = figure('name','Position 3D','units', 'normalized', 'outerposition', [0 1 1 1]);
% Create axes
axes1 = axes('Parent',fig_han,...
    'Position',[0.110393013100438 0.171280831210491 0.823908296943232 0.73182479175905]);

%axes1 = axes('Parent',fig_han,...
%    'Position',[0. 0.11128083121 0.12390 0.18182479175905]);

hold(axes1,'on');

%figure 
%set(gcf,'color','w');
%grid minor
%xlim([0 510])
plot3(px,py,pz,'LineWidth', 3,'color',color_point_traj);
hold on

plot3(M_VT_wind(:,5),M_VT_wind(:,6),M_VT_wind(:,7),'k', 'LineWidth', 2.5,'color',color_VTNMPC);
hold on

%plot3(M_PAMPC_wind(:,5),M_PAMPC_wind(:,6),M_PAMPC_nowind(:,7),'k', 'LineWidth', 2.5,'color',color_PAMPC);

hold on
plot3(M_NMPC_wind(2600:end,5),M_NMPC_wind(2600:end,6),M_NMPC_wind(2600:end,7),'k', 'LineWidth', 2.5,'color',color_NMPC);

hold on
patch(mx',my',mz','k','EdgeColor','w','FaceAlpha',0.2);

xlabel('{\it x}-axis (m)');
ylabel(['{\it y}-axis';'         (m)']);
zlabel('{\it z}-axis (m)');

%zlim([1.7 3.9]);





ax_han = gca;
set(ax_han,'FontSize',40)
leg_han = legend('point trajectory ','VT-NMPC','NMPC');
%leg_han = legend('point trajectory ','VT-NMPC','PAMPC','NMPC');
set(leg_han,'FontSize',35,'Location','northeast','Orientation','horizontal');
view(axes1,[-30.0836105158727 31.8655476563905]);
vec_pos = get(get(gca, 'XLabel'), 'Position');
set(get(gca, 'XLabel'), 'Position', vec_pos + [2 1.25 0.1]);
vec_pos = get(get(gca, 'YLabel'), 'Position');
set(get(gca, 'YLabel'), 'Position', vec_pos + [3.2 30 0]);
vec_pos = get(get(gca, 'ZLabel'), 'Position');
set(get(gca, 'ZLabel'), 'Position', vec_pos + [0.1 0 0]);
grid on
%saveas(gcf, [save_dest,'fig_pos_3D_sim_'], 'epsc');




%{
%% Plot 2-D Trajectory

figure('name','Position','units', 'normalized', 'outerposition', [0 0.266 0.659 0.934])
subplot(3,1,1)

plot([1:length(px)],px,'LineWidth', 2.5,'color',color_point_traj);
hold on
plot(t(1:skip:end),x(1:skip:end,1),'LineWidth', 2.5,'color',color_VTNMPC);

hold on
plot(t(1:skip:end),pa(1:skip:end,1),'LineWidth', 2.5,'color',color_PAMPC);
hold on
plot(t(1:skip:end),x_wp(1:skip:end,1),'LineWidth', 2.5,'color',color_GP);
xlabel('t [s]','FontSize',30);

%plot(time(1:skip:end),position_m(1:skip:end,1),'color',color_GP_NMPC);
ylabel('{\it x}-axis (m)','FontSize',30);
xlabel('time [s]','FontSize',30);
xlim([0 max(t)]);
%ylim([-1 30]);
ax_han = gca;
set(ax_han,'FontSize',30)
leg_han = legend('point traj','VT-NMPC','PAMPC','PID');
set(leg_han,'FontSize',30,'Location','northeast','Orientation','horizontal');
grid on

subplot(3,1,2)
plot(t(1:skip:end),p(1:skip:end,2),'LineWidth', 2.5,'color',color_point_traj);
hold on
plot(t(1:skip:end),x(1:skip:end,2),'LineWidth', 2.5,'color',color_VTNMPC);

hold on
plot(t(1:skip:end),pa(1:skip:end,2),'LineWidth', 2.5,'color',color_PAMPC);
hold on
plot(t(1:skip:end),x_wp(1:skip:end,2),'LineWidth', 2.5,'color',color_GP);

ylabel('{\it y}-axis (m)','FontSize',30);

xlabel('time [s]','FontSize',30);


xlim([t_start max(t)]);
%ylim([-2 1]);
ax_han = gca;
set(ax_han,'FontSize',30)
grid on

subplot(3,1,3)
plot(t(1:skip:end),p(1:skip:end,3),'LineWidth', 2.5,'color',color_point_traj);
hold on
plot(t(1:skip:end),x(1:skip:end,3),'LineWidth', 2.5,'color',color_VTNMPC);
hold on 
plot(t(1:skip:end),pa(1:skip:end,3),'LineWidth', 2.5,'color',color_PAMPC);
hold on
plot(t(1:skip:end),x_wp(1:skip:end,3),'LineWidth', 2.5,'color',color_GP);
hold on

ylabel('{\it z}-axis (m)','FontSize',30);
xlabel('time [s]','FontSize',30);

xlim([t_start max(t)]);

ax_han = gca;
set(ax_han,'FontSize',30)
grid on




%saveas(gcf, [save_dest,'fig_pos_sim_',num2str(wind_num)], 'epsc');




%{ 

%% Plot Costs

figure('name','Position','units', 'normalized', 'outerposition', [0 0.266 0.559 0.734])
%subplot(2,1,1)

plot(t(1:skip:end),C_h(1:skip:end,1),'LineWidth', 1.2,'color',color_Ch);
hold on
plot(t(1:skip:end),C_d(1:skip:end,1),'LineWidth', 2,'color',color_Cd);
hold on
plot(t(1:skip:end),C_r(1:skip:end,1),'LineWidth', 2,'color',color_Cr);
%hold on
%plot(t(1:skip:end),obj(1:skip:end,1),'LineWidth', 2,'color',color_Ct);

%plot(time(1:skip:end),position_m(1:skip:end,1),'color',color_GP_NMPC);
ylabel('Cost','FontSize',25);
xlabel('time [s]','FontSize',25);
xlim([t_start max(t)]);
%ylim([-1 30]);
ax_han = gca;
set(ax_han,'FontSize',25)
leg_han = legend('C_h','C_d','C_r','Objective');
set(leg_han,'FontSize',25,'Location','northeast','Orientation','horizontal');
grid on



%% Plot normals
figure('name','Position','units', 'normalized', 'outerposition', [0 0.266 0.559 0.734])

plot(t(1:skip:end),n(1:skip:end,1),'LineWidth', 2.5,'color',[1 0 0]);
hold on
plot(t(1:skip:end),n(1:skip:end,2),'LineWidth', 2.5,'color',[0 1 0]);
hold on
plot(t(1:skip:end),n(1:skip:end,3),'LineWidth', 2.5,'color',[0 0 1]);

%hold on
%plot(t(1:skip:end),x_ref(1:skip:end,1),'color',color_GP);

%plot(time(1:skip:end),position_m(1:skip:end,1),'color',color_GP_NMPC);
ylabel('magnitude','FontSize',25);
xlabel('time [s]','FontSize',25);
xlim([t_start max(t)]);


%ylim([-1 30]);
ax_han = gca;
set(ax_han,'FontSize',25)
leg_han = legend('n_x','n_y','n_z');
set(leg_han,'FontSize',25,'Location','northeast','Orientation','horizontal');
grid on

%% plot velocities
figure('name','Position','units', 'normalized', 'outerposition', [0 0.266 0.559 0.734])
plot(t(2:1:end),v_n,'color',color_VTNMPC);
ylabel('change in \itn','FontSize',30);
xlabel('time [s]','FontSize',30);
xlim([t_start max(t)]);

%% plot distance


for i=1:length(t)
    d_pa(i)=((p(i,1)-pa(i,1))^2+(p(i,2)-pa(i,2))^2+(p(i,3)-pa(i,3))^2)^0.5;
    %d_pid(i)
    d(i)=((p(i,1)-x(i,1))^2+(p(i,2)-x(i,2))^2+(p(i,3)-x(i,3))^2)^0.5;
end


figure('name','Position','units', 'normalized', 'outerposition', [0 0.266 0.559 0.734])
plot(t(1:skip*10:end),s_d(1:skip*10:end),'LineWidth', 2.5,'color',color_VTNMPC);
hold on
%plot(t(1:skip*10:end),d_pa(1:skip*10:end),'LineWidth', 2.5,'color',color_PAMPC);
%hold on
%plot(t(1:skip*10:end),d(1:skip*10:end),'LineWidth', 2.5,'color',color_GP);

yline(7.5,'--','LineWidth', 3.0,'Color',[0 0 0.1]);
hold on


yline(10,'--','LineWidth', 3.0);
hold on
%yline(8.5,'--','LineWidth', 3.0,'Color',[0 0 0.1]);
ylabel('distance [m]','FontSize',23);
xlabel('t [s]','FontSize',23);
xlim([t_start max(t)-50]);
ax_han = gca;
set(ax_han,'FontSize',25)
%ylim([7.2 10.3]);
%text(420,10.15,'maximum distance','FontSize',25)
%text(420,7.65,'safe distance','FontSize',25)
%text(420,8.65,'desired distance','FontSize',25)
%% plot KKT

figure('name','Position','units', 'normalized', 'outerposition', [0 0.266 0.559 0.734])
plot(t(1:skip*10:end),kkt(1:skip*10:end),'LineWidth', 2.5,'color',color_VTNMPC);
hold on
ylabel('KKT tolerance','FontSize',30);
xlabel('t [s]','FontSize',30);
xlim([t_start max(t)]);
ax_han = gca;
set(ax_han,'FontSize',25)


%% plot velocties
figure('name','Position','units', 'normalized', 'outerposition', [0 0.266 0.559 0.734])
subplot(3,1,1)

plot(t(1:skip:end),rates_d(1:skip:end,1),'color',color_VTNMPC);
%hold on
%plot(t(1:skip:end),(1:skip:end,1),'color',color_VTNMPC);
%hold on
%plot(t(1:skip:end),x_ref(1:skip:end,1),'color',color_GP);

%plot(time(1:skip:end),position_m(1:skip:end,1),'color',color_GP_NMPC);
ylabel('{\it roll_d} (rad/s)','FontSize',30);
xlim([t_start max(t)]);
%ylim([-1 30]);
ax_han = gca;
set(ax_han,'FontSize',30)
%leg_han = legend('point traj','VT-NMPC','GP');
%set(leg_han,'FontSize',28,'Location','northeast','Orientation','horizontal');
grid on

subplot(3,1,2)
plot(t(1:skip:end),rates_d(1:skip:end,2),'color',color_VTNMPC);
%hold on
%plot(t(1:skip:end),x(1:skip:end,2),'color',color_VTNMPC);
%hold on
%plot(t(1:skip:end),x_ref(1:skip:end,2),'color',color_GP);
ylabel('{\it Pitch_d} (rad/s)','FontSize',30);


xlim([t_start max(t)]);
%ylim([-2 1]);
ax_han = gca;
set(ax_han,'FontSize',30)
grid on

subplot(3,1,3)
plot(t(1:skip:end),rates_d(1:skip:end,3),'color',color_VTNMPC);
%hold on
%plot(t(1:skip:end),x(1:skip:end,3),'color',color_VTNMPC);
%hold on
%plot(t(1:skip:end),x_ref(1:skip:end,3),'color',color_GP);
ylabel('{\it yaw_d} (rad/s)','FontSize',30);


xlim([t_start max(t)]);

ax_han = gca;
set(ax_han,'FontSize',30)
grid on

%% plot control effort 
writematrix(x_wp, "x_wp.txt");

%}
%}


%% Distance
d_VT_w=((M_VT_wind(:,5)-M_VT_wind(:,8)).^2+(M_VT_wind(:,6)-M_VT_wind(:,9)).^2+(M_VT_wind(:,7)-M_VT_wind(:,10)).^2).^0.5;

d_NMPC_w=((M_NMPC_wind(:,5)-M_NMPC_wind(:,8)).^2+(M_NMPC_wind(:,6)-M_NMPC_wind(:,9)).^2+(M_NMPC_wind(:,7)-M_NMPC_wind(:,10)).^2).^0.5;
figure
plot ([1:length(d_VT_w)-2250]/50,d_VT_w(2250:end-1),'LineWidth', 2,'color',color_VTNMPC)
hold on
plot ([1:length(d_NMPC_w)-2700]/50,d_NMPC_w(2700:end-1),'LineWidth', 2,'color',color_NMPC)
xlim([0 30000/50])
%legend ('VTNMPC' ,'NMPC')

leg_han = legend('VTNMPC' ,'NMPC');
set(leg_han,'FontSize',30,'Location','northeast','Orientation','horizontal');
xlabel('time [s]');
ylabel('distance [m]');
ax_han = gca;
set(ax_han,'FontSize',30)

grid on
