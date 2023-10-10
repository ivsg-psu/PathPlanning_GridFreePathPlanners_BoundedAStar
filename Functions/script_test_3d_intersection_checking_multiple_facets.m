clear all; close all; clc

verts = [1 1 0; 2 1 0;  3 1 20; 2 1 20]; % a line that translates its length in x over the course of 20 seconds
verts2 = [2 2 10; 3 2 10;  4 2 30; 3 2 30]; % a line that translates its length in x over the course of 20 seconds
start = [2 0 0];
finish = [4 4 45];
figure; hold on; box on; title('surfels and line from start to goal');
fig = gcf;

% remove unnecessary white space
xlabel('x [m]')
ylabel('y [m]')
zlabel('t [s]')
fill3(verts(1:3,1),verts(1:3,2),verts(1:3,3),'b');
fill3(verts([1,3,4],1),verts([1,3,4],2),verts([1,3,4],3),'r');
fill3(verts2(1:3,1),verts2(1:3,2),verts2(1:3,3),'g');
fill3(verts2([1,3,4],1),verts2([1,3,4],2),verts2([1,3,4],3),'y');
plot3(start(1),start(2),start(3),'gx');
plot3(finish(1),finish(2),finish(3),'rx');
plot3([start(1) finish(1)],[start(2) finish(2)],[start(3) finish(3)])
% finish = start + dir
% thus dir = finish-start
[intersect, t, u, v, xcoor] = TriangleRayIntersection (start, finish-start, verts(1,:),verts(2,:),verts(3,:),'lineType','segment','border','exclusive')
[intersect2, t2, u2, v2, xcoor2] = TriangleRayIntersection (start, finish-start, verts(1,:),verts(3,:),verts(4,:),'lineType','segment')
[intersect3, t3, u3, v3, xcoor3] = TriangleRayIntersection (start, finish-start, verts2(1,:),verts2(2,:),verts2(3,:),'lineType','segment','border','exclusive')
[intersect4, t4, u4, v4, xcoor4] = TriangleRayIntersection (start, finish-start, verts2(1,:),verts2(3,:),verts2(4,:),'lineType','segment','border','exclusive')
plot3(xcoor(1),xcoor(2),xcoor(3),'cx')
plot3(xcoor3(1),xcoor3(2),xcoor3(3),'cx')
plot3(xcoor4(1),xcoor4(2),xcoor4(3),'cx')
legend('tri 1','tri 2','tri3','tri4','start','goal','','intersection')
