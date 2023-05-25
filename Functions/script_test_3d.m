clear all; close all; clc

polytopes(1).vertices = [1 1 0; 2 1 0;  3 1 20; 2 1 20]; % a line that translates its length in x over the course of 20 seconds
start = [2 0 0];
finish = [2 2 15];
fill3(polytopes(1).vertices(:,1),polytopes(1).vertices(:,2),polytopes(1).vertices(:,3),'b');
box on; hold on;
plot3(start(1),start(2),start(3),'gx');
plot3(finish(1),finish(2),finish(3),'rx');

https://www.mathworks.com/matlabcentral/fileexchange/33073-triangle-ray-intersection
https://en.wikipedia.org/wiki/Intersection_of_a_polyhedron_with_a_line
https://www.mathworks.com/help/matlab/visualize/multifaceted-patches.html
% at each time t, calculate P
% ir for all points on all bodies that are candidates for collision
% select one point P
% jr on body j and test if it is inside body i
% cast an infinite ray in any direction from P
% jr and compute intersections with all facets on body i
% if there are an even number of intersections, P
% jr is not inside body i
% if there are an odd number of intersections, P
% jr is inside body i
% works for concavities, holes and self-crossing and is independent of CW versus CCW boundary
% ray-facet intersection is similar to edge-facet intersection described below
% point in polygon does not always work for thin bodies (may need edge intersection)
%
