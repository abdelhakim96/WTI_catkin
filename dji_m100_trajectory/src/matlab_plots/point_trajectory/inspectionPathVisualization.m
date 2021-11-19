%% inspectionPathVisualization.m
% 
% This script was written by Andreas Bircher on 6th October 2014
%         andreas.bircher@mavt.ethz.ch / bircher@gmx.ch
% 
% Scenarios for autonomous inspection are visualized, for which the path is
% computed using the provided planner.
% 
%% 
clear all; close all;
run('inspectionScenario');


for i = 1:162
       
        b1(1)=(meshX(i,3)-meshX(i,1));
        b1(2)=(meshY(i,3)-meshY(i,1));
        b1(3)=(meshZ(i,3)-meshZ(i,1));
        
        px(i)=(meshX(i,3)+meshX(i,2)+meshX(i,3))/3;
        py(i)=(meshY(i,3)+meshY(i,2)+meshY(i,3))/3;
        pz(i)=(meshZ(i,3)+meshZ(i,2)+meshZ(i,3))/3;
        
        b2(1)=(meshX(i,3)-meshX(i,2));
        b2(2)=(meshY(i,3)-meshY(i,2));
        b2(3)=(meshZ(i,3)-meshZ(i,2));
        
 
    
       a1(i,:) = 0.5*cross(b1,b2);
end 

px=px';
py=py';
pz=pz';


set(0,'defaultfigurecolor',[1 1 1])
handle = figure;



plot3(inspectionPath(:,1),inspectionPath(:,2),inspectionPath(:,3),'b', 'LineWidth', 2.0,'LineSmoothing', 'on');
hold on;

quiver3(px,py,pz,a1(:,1),a1(:,2),a1(:,3),'g')

patch(meshX',meshY',meshZ','r','EdgeColor','k','FaceAlpha',0.7);
xlabel('x[m]');
ylabel('y[m]');
zlabel('z[m]');
if(numObstacles>0)
    for i = 1:numObstacles
        setBox(handle, obstacle{i}(2,1), obstacle{i}(2,2), obstacle{i}(2,3), obstacle{i}(1,1), obstacle{i}(1,2), obstacle{i}(1,3));
    end
end
hold off;
legend('Optimal Path','Inspected Wind Turbine Blades')
title(['Global Planner Inspection Path ']);
axis equal;

