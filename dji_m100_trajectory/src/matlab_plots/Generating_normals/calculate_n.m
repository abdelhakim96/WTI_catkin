%% inspectionPathVisualization.m
% 
% This script was written by Andreas Bircher on 6th October 2014
%         andreas.bircher@mavt.ethz.ch / bircher@gmx.ch
% 
% Scenarios for autonomous inspection are visualized, for which the path is
% computed using the provided planner.
% 
%% 
% this->a = ((this->x2-this->x1).cross(this->x3-this->x2))/2;
clear all
clc

mx = dlmread('MeshX.txt');
my = dlmread('MeshY.txt');
mz = dlmread('MeshZ.txt');
path=dlmread('path.txt');
mxx=mx;
for i=1:length(path)/2
   path(i+1,:)=[]; 
end
path(:,4:6)=[];
path_i=path;
px=(mx(:,1)+mx(:,2)+mx(:,3))/3;
py=(my(:,1)+my(:,2)+my(:,3))/3;
pz=(mz(:,1)+mz(:,2)+mz(:,3))/3;



v1=[mx(:,1) my(:,1) mz(:,1)];
v2=[mx(:,2) my(:,2) mz(:,2)];

v3=[mx(:,3) my(:,3) mz(:,3)];

for i=1:length(mx)
    
        n(i,:)=cross((v2(i,:)-v1(i,:)),(v3(i,:)-v2(i,:)))/2;
        %n(i,:)=n(i,:)/(n(i,1)^2+n(i,2)^2+n(i,3)^2)^0.5;
        n(i,3)=n(i,3)/(n(i,1)^2+n(i,2)^2+n(i,3)^2)^0.5;
        n(i,1:2)=n(i,1:2)/(n(i,1)^2+n(i,2)^2)^0.5;
     
        %cross((mx(i,2)-mx(i,1)),(mx(i,3)-mx(i,2)))/2;
       % ax(i)=cross((mx(i,2)-mx(i,1)),(mx(i,3)-mx(i,2)))/2;
      %  ay(i)=cross((my(i,2)-my(i,1)),(my(i,3)-my(i,2))/2;
       % az(i)=cross((mz(i,2)-mz(i,1)),(mz(i,3)-mz(i,2)))/2;
        
    
end
nx=n(:,1);
ny=n(:,2);
nz=n(:,3);

nx_160=n(:,1);
ny_160=n(:,2);
nz_160=n(:,3);

d=1000;
nn=[];
aa=1;
for i=1:length(path)
for j=1:length(path)
    
    d1= ((path(i,1)-px(j))^2+(path(i,2)-py(j))^2+(path(i,3)-pz(j))^2)^0.5;
    d2= ((px(j)-px(aa))^2+(py(j)-py(aa))^2+(pz(j)-pz(aa))^2)^0.5;
    %d1= (Path(2*i-1,2)-y(j))^2;
    %d1=(path_new(i,2)-py(j))^2;
    
    if (d1-0)^2<(d-0)^2 
        d=d1;
        n=j;
        
    end
    
    
   %if ismember(j, nn)==0
   %    n=aa+1; 
   %end

end 

    



nn(i)=n;
aa=n;
dd(i)=d;
x1(i)=px(n);
y1(i)=py(n);
z1(i)=pz(n);
nx_o(i)=nx(n);
ny_o(i)=ny(n);
nz_o(i)=nz(n);
mx_o(i,:)=mx(n,:);
my_o(i,:)=my(n,:);
mz_o(i,:)=mz(n,:);

d=100000;
n=0;
end





px=x1;
py=y1;
pz=z1;
nx=nx_o;
ny=ny_o;
nz=nz_o;
mx=mx_o;
my=my_o;
mz=mz_o;
px_160=px;
py_160=py;
pz_160=pz;

mesh_x=mx_o;
mesh_y=my_o;
mesh_z=mz_o;



for j=1:5
for  i=1:length(px)-1
    px1(2*i-1)=px(i);
    px1(2*i)=(px(i)+px(i+1))/2;
    
    px1(2*i+1)=px(i+1);
    
    py1(2*i-1)=py(i);
    py1(2*i)=(py(i)+py(i+1))/2;
    
    py1(2*i+1)=py(i+1);
    
    pz1(2*i-1)=pz(i);
    pz1(2*i)=(pz(i)+pz(i+1))/2;
    
    pz1(2*i+1)=pz(i+1);
    
    
    nx1(2*i-1)=nx(i);
    nx1(2*i)=(nx(i)+nx(i+1))/2;
    
    nx1(2*i+1)=nx(i+1);
    
    ny1(2*i-1)=ny(i);
    ny1(2*i)=(ny(i)+ny(i+1))/2;
    
    ny1(2*i+1)=ny(i+1);
    
    nz1(2*i-1)=nz(i);
    nz1(2*i)=(nz(i)+nz(i+1))/2;
    
    nz1(2*i+1)=nz(i+1);
    
    
    path1(2*i-1,:)= path(i,:);
    path1(2*i,:)=( path(i,:)+ path(i+1,:))/2;
    path1(2*i+1,:)= path(i+1,:);
    
    
    
    mx1(2*i-1,:)=mx(i,:);
    mx1(2*i,:)=(mx(i,:)+mx(i+1,:))/2;
    
    mx1(2*i+1,:)=mx(i+1,:);
    
    my1(2*i-1,:)=my(i,:);
    my1(2*i,:)=(my(i,:)+my(i+1,:))/2;
    
    my1(2*i+1,:)=my(i+1,:);
    
    mz1(2*i-1,:)=mz(i,:);
    mz1(2*i,:)=(mz(i,:)+mz(i+1,:))/2;
    
    mz1(2*i+1,:)=mz(i+1,:);
    
    
    
    
    if i==length(px)-1
        path=path1;
        px=px1;
        py=py1;
        pz=pz1;
        nx=nx1;
        ny=ny1;
        nz=nz1;
        mx=mx1;
        my=my1;
        mz=mz1;
        
    end
    
 
end
end


writematrix(px', "px.txt");
writematrix(py', "py.txt");
writematrix(pz', "pz.txt");
writematrix(nx', "nx_160.txt");
writematrix(ny', "ny_160.txt");
writematrix(nz', "nz_160.txt");

writematrix(n, "n.txt");

writematrix(mx', "mx.txt");

writematrix(my', "my.txt");

writematrix(mz', "mz.txt");


writematrix(px_160', "px_160.txt");
writematrix(py_160', "py_160.txt");
writematrix(pz_160', "pz_160.txt");

writematrix(nx_o', "nx_160.txt");
writematrix(ny_o', "ny_160.txt");
writematrix(nz_o', "nz_160.txt");


%writematrix(mesh_x', "mesh_x.txt");

%writematrix(mesh_y', "mesh_y.txt");

%writematrix(mesh_z', "mesh_z.txt");


writematrix(mx_o', "mesh_x.txt");

writematrix(my_o', "mesh_y.txt");

writematrix(mz_o', "mesh_z.txt");
    


%run('inspectionScenario');
set(0,'defaultfigurecolor',[1 1 1])
handle = figure;
plot3(path(:,1),path(:,2),path(:,3),'b', 'LineWidth', 2.0,'LineSmoothing', 'on');
hold on
quiver3(px_160',py_160',pz_160',nx_o',ny_o',nz_o','g')
%quiver3(px',py',pz',nx',ny',nz','g')

hold on;
patch(mx_o',my_o',mz_o','r','EdgeColor','k','FaceAlpha',0.7);

hold on

plot3(px,py,pz,'k', 'LineWidth', 2.0,'LineSmoothing', 'on');

xlabel('x[m]');
ylabel('y[m]');
zlabel('z[m]');

legend('Optimal Path','Inspected Wind Turbine Blades')
title(['Global Planner Inspection Path ']);
axis equal;




