clear all
clc

set(gcf,'color','w');
M = dlmread('inspection_1.txt');


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


%plot3(M(1321:26954,2),M(1321:26954,3),M(1321:26954,4),'k', 'LineWidth', 3.0,'LineSmoothing', 'on');

hold on

%plot3(M(1321:26954,11),M(1321:26954,12),M(1321:26954,13),'r', 'LineWidth', 2.0,'LineSmoothing', 'on');

hold on 
%scatter3(M(1321:500:26954,5),M(1321:500:26954,6),M(1321:500:26954,7), 'k');

%scatter3(a2,b2,c2, 10, e, 'filled')

%legend('desired path','points to view','obtained path')
%cb = colorbar(); 
%title(cb, 'Error')


%figure

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

xlim([0 510])




