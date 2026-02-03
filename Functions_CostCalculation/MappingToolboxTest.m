% TEST: make a map containing State College and Ann Arbor and overlay a
% wind field

fig_num = 10001;

% Define locations + plot
StateCollegeCoords = [40.79445 -77.861639];
AnnArborCoords = [42.281372 -83.748462];

LLData = [StateCollegeCoords; AnnArborCoords];
%plotFormat.LineStyle = 'none';
plotFormat = [];

h_geoplot = fcn_plotRoad_plotLL((LLData), (plotFormat), (fig_num));
[latlim, lonlim] = geolimits('auto');

% Define wind field for specified range
XY_range = [lonlim(1), latlim(1), lonlim(2), latlim(2)];
NpointsInSide = [];
windMagnitude = [];
randomSeed = 1001;
peaksMode = 0;

[windFieldU, windFieldV, windFieldX, windFieldY] = fcn_BoundedAStar_fillWindField(XY_range, NpointsInSide, windMagnitude, randomSeed, peaksMode, -1);

% Process wind field to make it plottable on geoaxes
magField = sqrt(windFieldU.^2 + windFieldV.^2);
[X, Y] = meshgrid(windFieldX, windFieldY);
levels = linspace(0, 256, 10);

lat0 = mean(latlim);
lon0 = mean(lonlim);

[contourLines, contourPolygons] = geocontourxy(X,Y,magField, lat0, lon0, 500);

figure(125546)
hold on

cmap = jet(1 + length(levels));
for k = 1:length(contourPolygons)
      lat = contourPolygons(k).Latitude;
      lon = contourPolygons(k).Longitude;
      
      geoshow(lat,lon,'Display','polygon', ...
             'FaceColor',cmap(k,:),'FaceAlpha',0.5,'EdgeColor','none')
end

%fcn_BoundedAStar_plotWindField(windFieldU, windFieldV, windFieldX, windFieldY, 'default', fig_num);