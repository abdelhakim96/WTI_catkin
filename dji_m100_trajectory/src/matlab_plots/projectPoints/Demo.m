%Quick Program to demo the use of projectPoints


x =dlmread('px.txt');
y =dlmread('py.txt');
z =dlmread('pz.txt');


%% generate a set of 3d points
%z = peaks;
%x = repmat(1:size(z,1),size(z,1),1);
%y = x';
c = z - min(z(:));
c = c./max(c(:));
c = round(255*c) + 1;
cmap = colormap(jet(256));
c = cmap(c,:);

points = [x(:),y(:),z(:),c];

%% setup 

%setup camera with focal length 200, centre 500,500
cam = [500,0,500;0,500,500;0,0,1];

%setup image
imageSize = [1000,1000];
%x=y 
%y=x
%z=z;
%create a tform matrix
angles = [270,0,0]*pi/180;
%position = [-25,-25,70];
position = [0 -300 250];
tform = eye(4);
th=-angles(1);
%tform(1:3,1:3) = angle2dcm(angles(1),angles(2),angles(3));

tform(1:3,1:3)=[cos(th) -sin(th) 0 ; sin(th) cos(th) 0 ;  0 0 1];
%tform(1:3,1:3)=eye(3);
tform(1:3,4) = position;

%add a little lens distortion
dist = [0.1,0.005];
%dist =[0,0];
%project the points into image coordinates
[projected, valid] = projectPoints(points, cam, tform, dist, imageSize,true);
projected = projected(valid,:);

%show the projection
subplot(1,2,1);
scatter3(points(:,1),points(:,2),points(:,3),20,points(:,4:6),'fill');
axis equal;
title('Original Points');

subplot(1,2,2);
scatter(projected(:,1),projected(:,2),20,projected(:,3:5),'fill');
axis equal;
title('Points projected with camera model');