clear all
clc

set(gcf,'color','w');
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

e1=a1-a2;
e2=b1-b2;
e3=c1-c2;


plot3(M(1321:26954,2),M(1321:26954,3),M(1321:26954,4),'k', 'LineWidth', 3.0,'LineSmoothing', 'on');

hold on




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

subplot(3,2,2)

plot(M(1321:26954,1)-M(1321,1),e1,'r', 'LineWidth', 1.0,'LineSmoothing', 'on');


xlabel('time (s)') 
ylabel('error x-axis (m)')
xlim([0 510])

subplot(3,2,4)



hold on
xlim([0 510])

plot(M(1321:26954,1)-M(1321,1),e2,'r', 'LineWidth', 1.0,'LineSmoothing', 'on');

xlabel('time (s)') 
ylabel('error y-axis (m)') 
subplot(3,2,6)

plot(M(1321:26954,1)-M(1321,1),e3,'r', 'LineWidth', 1.0,'LineSmoothing', 'on');

xlabel('time (s)') 
ylabel('error z-axis (m)') 

xlim([0 510])


subplot(3,2,1)

plot(M(1321:26954,1)-M(1321,1),M(1321:26954,11),'b', 'LineWidth', 1.0,'LineSmoothing', 'on');
hold on
plot(M(1321:26954,1)-M(1321,1),M(1321:26954,2),'k', 'LineWidth', 1.0,'LineSmoothing', 'on');

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

xlabel('time (s)') 
ylabel('y-axis (m)') 
legend('NMPC','REF')
subplot(3,2,5)

plot(M(1321:26954,1)-M(1321,1),M(1321:26954,13),'b', 'LineWidth', 1.0,'LineSmoothing', 'on');
hold on
plot(M(1321:26954,1)-M(1321,1),M(1321:26954,4),'k', 'LineWidth', 1.0,'LineSmoothing', 'on');
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
subplot(3,2,4)

xlim([0 510])


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


figure
plot(M(1321:26954,1)-M(1321,1),psi,'b', 'LineWidth', 2.0,'LineSmoothing', 'on');
hold on
plot(M(1321:26954,1)-M(1321,1),psi_d,'r', 'LineWidth', 2.0,'LineSmoothing', 'on');

xlabel('time (s)') 
ylabel('Psi vs Psi_d') 

xlim([0 510])





figure
plot(M(1321:500:26954,1)-M(1321,1),s(1:500:end),'b', 'LineWidth', 2.0,'LineSmoothing', 'on');

xlabel('time (s)') 
ylabel('Point to view Objective') 

xlim([0 510])


figure

hold on
plot(M(1321:500:26954,1)-M(1321,1),C(1:500:end),'b', 'LineWidth', 2.0,'LineSmoothing', 'on');
hold on
plot(M(1321:500:26954,1)-M(1321,1),C1(1:500:end),'r', 'LineWidth', 2.0,'LineSmoothing', 'on');


xlabel('time (s)') 
ylabel('Coverage ') 

xlim([0 510])



C1(1:end)=C1(1:end);
%coverage=eye(,5;
coverage(:,1)=psi;
coverage(:,2)=M(1321:26954,5);
coverage(:,3)=M(1321:26954,6);
coverage(:,4)=M(1321:26954,11);
coverage(:,5)=M(1321:26954,12);


figure
plot3(M(1321:26954,11),M(1321:26954,12),M(1321:26954,13),'k', 'LineWidth', 2.0,'LineSmoothing', 'on');
hold on
psi(1:end)=psi(1:end)*pi/180;
quiver3(M(1321:500:26954,11),M(1321:500:26954,12),M(1321:500:26954,13),5*cos(psi(1:500:end)),5*sin(psi(1:500:end)),zeros(size(psi(1:500:end))),'off','g')

C1=C1/3;
hold on
scatter3(M(1321:500:26954,5),M(1321:500:26954,6),M(1321:500:26954,7),10,1-C1(1:500:end),'filled','SizeData',20);
hold on


%set( cb2, 'YDir', 'reverse' );
cb2=colorbar();
set( cb2, 'YDir', 'reverse' );
%cb2=flipud(cb1);
%set( cb2, 'YDir', 'reverse' );
title(cb2, 'Coverage')
set( cb2, 'YDir', 'reverse' );
legend('NMPC trajectory','Drone Heading','Point to View')

colorbar('Ticks',[0,0.2,0.4,0.6,0.8],...
         'TickLabels',{'100%','80%','60%','40%','20%'})

%dlmwrite('input_coverage.txt',coverage(:),'newline','pc')
writematrix(coverage,'myData.txt','Delimiter',';')  
type myData.txt
