clear all
clc

set(gcf,'color','w');
%M = dlmread('inspection_1.txt');
M = dlmread('inspection_1.txt');
C = dlmread('coverage.txt');
C1 = dlmread('hakim.txt');
a1=M(1321:26954,2);

b1=M(1321:26954,3);
c1=M(1321:26954,4);

a2=M(1321:26954,11);



b2=M(1321:26954,12);
c2=M(1321:26954,13);

e=((a1-a2).^2+(b1-b2).^2+(c1-c2).^2).^0.5;
e=abs(e);
e1=a1-a2;
e2=b1-b2;
e3=c1-c2;

e1=abs(e1);
e2=abs(e2);
e3=abs(e3);

v1=M(1321:26954,14);
v2=M(1321:26954,15);
v3=M(1321:26954,16);
v=(v1.^2+v2.^2+v3.^2).^0.5;
grid on
set(gcf,'color','w');
figure
%plot3(M(1321:26954,8),M(1321:26954,9),M(1321:26954,10),'k', 'LineWidth', 3.0,'LineSmoothing', 'on');
set(gcf,'color','w');
grid minor
plot(M(1321:26954,1)-M(1321,1),v(1:end),'b', 'LineWidth', 2.0,'LineSmoothing', 'on');
xlim([0 510])


xlabel('time (s)') 
ylabel('velocity (m/s)')
grid minor

figure

hold on


grid on

%plot3(M(1321:26954,11),M(1321:26954,12),M(1321:26954,13),'r', 'LineWidth', 2.0,'LineSmoothing', 'on');

hold on 
%scatter3(M(1321:500:26954,5),M(1321:500:26954,6),M(1321:500:26954,7), 'o','k');

scatter3(a2,b2,c2, 10, e, 'filled')


cb1 = colorbar(); 
title(cb1, 'Trajectory Error')

%scatter3(a2,b2,c2, 10, C1, 'filled')

legend('desired path','points to view','obtained path')

xlabel('x-axis (m)') 
ylabel('y-axis (m)')
zlabel('z-axis (m)')







figure

%plot(M(1321:26954,1),M(1321:26954,5),'k', 'LineWidth', 3.0,'LineSmoothing', 'on');





hold on
set(gcf,'color','w');
grid minor
subplot(3,2,2)

plot(M(1321:26954,1)-M(1321,1),e1,'r', 'LineWidth', 1.0,'LineSmoothing', 'on');
grid minor

xlabel('time (s)') 
ylabel('error x-axis (m)')
xlim([0 510])

subplot(3,2,4)

grid minor

hold on
xlim([0 510])
grid minor
plot(M(1321:26954,1)-M(1321,1),e2,'r', 'LineWidth', 1.0,'LineSmoothing', 'on');

xlabel('time (s)') 
ylabel('error y-axis (m)') 
subplot(3,2,6)

plot(M(1321:26954,1)-M(1321,1),e3,'r', 'LineWidth', 1.0,'LineSmoothing', 'on');
grid minor
xlabel('time (s)') 
ylabel('error z-axis (m)') 

xlim([0 510])
grid minor

subplot(3,2,1)
grid minor
plot(M(1321:26954,1)-M(1321,1),M(1321:26954,11),'b', 'LineWidth', 1.0,'LineSmoothing', 'on');
hold on
plot(M(1321:26954,1)-M(1321,1),M(1321:26954,2),'k', 'LineWidth', 1.0,'LineSmoothing', 'on');


plot(M(1321:26954,1)-M(1321,1),M(1321:26954,5),'r', 'LineWidth', 1.0,'LineSmoothing', 'on');


xlabel('time (s)') 
ylabel('x-axis (m)')
xlim([0 510])

legend('NMPC','REF')
subplot(3,2,3)


plot(M(1321:26954,1)-M(1321,1),M(1321:26954,12),'b', 'LineWidth', 1.0,'LineSmoothing', 'on');
legend('NMPC','REF')

hold on
xlim([0 510])

plot(M(1321:26954,1)-M(1321,1),M(1321:26954,3),'k', 'LineWidth', 1.0,'LineSmoothing', 'on');
plot(M(1321:26954,1)-M(1321,1),M(1321:26954,6),'r', 'LineWidth', 1.0,'LineSmoothing', 'on');

xlabel('time (s)') 
ylabel('y-axis (m)') 
legend('NMPC','REF')
subplot(3,2,5)

plot(M(1321:26954,1)-M(1321,1),M(1321:26954,13),'b', 'LineWidth', 1.0,'LineSmoothing', 'on');
hold on
plot(M(1321:26954,1)-M(1321,1),M(1321:26954,4),'k', 'LineWidth', 1.0,'LineSmoothing', 'on');

plot(M(1321:26954,1)-M(1321,1),M(1321:26954,7),'r', 'LineWidth', 1.0,'LineSmoothing', 'on');
legend('NMPC','REF')
xlabel('time (s)') 
ylabel('z-axis (m)') 
M(1321:26954,1);

px=M(1321:26954,2);
py=M(1321:26954,3);
pz=M(1321:26954,4);


x=M(1321:26954,11);
y=M(1321:26954,12);
z=M(1321:26954,13);

n1=px-x;
n2=py-y;


theta=M(1321:26954,17);
phi=M(1321:26954,18);
psi=M(1321:26954,19);


for i=1:length(psi)
    
if n1(i)>0
psi_d(i)=atan(n2(i)/n1(i))*(180/pi);


else
 psi_d(i)=-180+atan(n2(i)/n1(i))*(180/pi); 
end

end



nn=(n1.^2+n2.^2).^0.5;
s=(cos(psi).*n1+sin(psi).*n2)./nn;


figure 
set(gcf,'color','w');

subplot(3,2,2)

plot(M(1321:26954,1)-M(1321,1),M(1321:26954,20),'b', 'LineWidth', 1.0,'LineSmoothing', 'on');

xlabel('time (s)') 
ylabel('Roll Rate (deg/s)') 

xlim([0 510])
subplot(3,2,4)




plot(M(1321:26954,1)-M(1321,1),M(1321:26954,21),'b', 'LineWidth', 1.0,'LineSmoothing', 'on');

xlabel('time (s)') 
ylabel('Pitch Rate (deg/s)') 

xlim([0 510])

subplot(3,2,6)

plot(M(1321:26954,1)-M(1321,1),M(1321:26954,22),'b', 'LineWidth', 1.0,'LineSmoothing', 'on');

xlabel('time (s)') 
ylabel('Yaw Rate (deg/s)') 
xlim([0 510])





subplot(3,2,1)

plot(M(1321:26954,1)-M(1321,1),theta,'k', 'LineWidth', 1.0,'LineSmoothing', 'on');

xlabel('time (s)') 
ylabel('Roll (deg)') 
xlim([0 510])

subplot(3,2,3)


plot(M(1321:26954,1)-M(1321,1),phi,'k', 'LineWidth', 1.0,'LineSmoothing', 'on');

xlabel('time (s)') 
ylabel('Pitch (deg)') 
xlim([0 510])


subplot(3,2,5)

plot(M(1321:26954,1)-M(1321,1),psi,'k', 'LineWidth', 1.0,'LineSmoothing', 'on');

xlabel('time (s)') 
ylabel('Yaw (deg)') 

xlim([0 510])
set(gcf,'color','w');


figure
plot(M(1321:26954,1)-M(1321,1),psi,'b', 'LineWidth', 2.0,'LineSmoothing', 'on');
hold on
plot(M(1321:26954,1)-M(1321,1),psi_d,'r', 'LineWidth', 2.0,'LineSmoothing', 'on');

xlabel('time (s)') 
ylabel('Psi vs Psi_d') 

xlim([0 510])


psi(1:end)=psi(1:end)*pi/180;
set(gcf,'color','w');


figure
plot(M(1325:500:26954,1)-M(1325,1),s(4:500:end),'b', 'LineWidth', 2.0,'LineSmoothing', 'on');

xlabel('time (s)') 
ylabel('Point to view Objective') 

xlim([0 510])

set(gcf,'color','w');

figure

hold on
%plot(M(1321:500:26954,1)-M(1321,1),C(1:500:end),'b', 'LineWidth', 2.0,'LineSmoothing', 'on');
hold on
plot(M(1325:500:26954,1)-M(1325,1),C1(5:500:end)*100/3,'k', 'LineWidth', 2.0,'LineSmoothing', 'on');


xlabel('time (s)') 
ylabel('Visibility of point k (%) ') 

xlim([0 510])



C1(1:end)=C1(1:end);
%coverage=eye(,5;
coverage(:,1)=psi;
coverage(:,2)=M(1321:26954,5);
coverage(:,3)=M(1321:26954,6);
coverage(:,4)=M(1321:26954,11);
coverage(:,5)=M(1321:26954,12);

set(gcf,'color','w');


figure
set(gcf,'color','w');


plot3(M(1321:26954,11),M(1321:26954,12),M(1321:26954,13),'k', 'LineWidth', 4.0,'LineSmoothing', 'on');
hold on

quiver3(M(1321:500:26954,11),M(1321:500:26954,12),M(1321:500:26954,13),5*cos(psi(1:500:end)),5*sin(psi(1:500:end)),zeros(size(psi(1:500:end))),'off','g','linewidth',3)

C1=C1/3;
hold on
scatter3(M(1321:500:26954,11),M(1321:500:26954,12),M(1321:500:26954,13),200,1-C1(1:500:end),'filled','SizeData',200);
hold on




%point
scatter3(M(1321+500*16,11),M(1321+500*16,12),M(1321+500*16,13),200,1-C1(1321+500*16),'filled','SizeData',200);

scatter3(M(1321+500*16,5),M(1321+500*16,6),M(1321+500*16,7),200,1-C1(1321+500*16),'filled','SizeData',400);

%scatter3(M(1321:1:26954,11),M(1321:1:26954,12),M(1321:1:26954,13),200,e,'filled','SizeData',10);
hold on


labels = {'label 1','label 2'};
hold on
%scatter3(M(1321,11),M(1321,12),M(1321,13),40, '*','b');
hold on
%scatter3(M(26954,11),M(26954,12),M(26954,13),40, '*','r');
hold on
%scatter3(M(1321+150,11),M(1321+150,12),M(1321+150,13),100, 'o','r');
hold on
%scatter3(M(1321+150,5),M(1321+150,6),M(1321+150,7),100, 'o','r');
hold on
%quiver3(M(1321+150,11),M(1321+150,12),M(1321+150,13),5*cos(psi(1321+150)),5*sin(psi(1321+150)),zeros(size(psi(1321+150))),'off','r')
%text(x,y,labels,'VerticalAlignment','bottom','HorizontalAlignment','right')


%scatter3(M(1321+460,11),M(1321+460,12),M(1321+460,13),100, 'o','r');
hold on
%scatter3(M(1321+460,5),M(1321+460,6),M(1321+460,7),100, 'o','r');
hold on
%quiver3(M(1321+460,11),M(1321+460,12),M(1321+460,13),5*cos(psi(1321+460)),5*sin(psi(1321+460)),zeros(size(psi(1321+460))),'off','r')







%set( cb2, 'YDir', 'reverse' );
cb2=colorbar();
set( cb2, 'YDir', 'reverse' );
%cb2=flipud(cb1);
%set( cb2, 'YDir', 'reverse' );
title(cb2, 'Coverage')
set( cb2, 'YDir', 'reverse' );
legend('NMPC trajectory','Drone Heading','Point to View','start', 'end' )

colorbar('Ticks',[0,0.2,0.4,0.6,0.8],...
         'TickLabels',{'100%','80%','60%','40%','20%'})

%dlmwrite('input_coverage.txt',coverage(:),'newline','pc')
writematrix(coverage,'myData.txt','Delimiter',';')  
type myData.txt

figure

s1=(cos(psi).*n1+sin(psi).*n2)./nn;


plot(M(1325:500:26954,1)-M(1325,1),s1(4:500:end),'b', 'LineWidth', 3.0,'LineSmoothing', 'on');




