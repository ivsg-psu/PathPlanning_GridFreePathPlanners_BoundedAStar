function [preExpansionPoints, flagPointsFilledArtificially] = ...
    fcn_BoundaryXP_prepareStartPoints(startPoints, radius)

% Keep only the unique (non-repeating) points. Do not resort them.
uniqueStartPoints = unique(startPoints,'rows','stable');

% Fill in some variables that are used for calculations that follow, for
% cases with 1 or 2 points
Npoints = length(uniqueStartPoints(:,1));
if Npoints==1 || Npoints==2
    % Define the number of directions to be considered from each value
    % of the x0 initial set, if the initial set contains only 1 or 2 points
    % Number of directions for initial expansion. Note that the first and last
    % points are the same, so effectively this divides 360 degrees by
    % (Ndirections-1)
    Ndirections = 181; % N = 7 gives 6-sided hexagon
    angles = linspace(0,2*pi,Ndirections)';
    preExpansionRadius  = (0.0001*radius); 
    flagPointsFilledArtificially = 1;
else
    flagPointsFilledArtificially = 0;
end

%%%%
% Make sure that the points form an enclosed area.
% The following takes small perturbations away from the uniqueStartPoints.
% There are three cases:
% 1) There is 1 point - in this case, a very small expansion is made in all
%    directions to create an enclosed space
% 2) There are 2 points - in this case, the expansion is away from both
%    ends, creating an enclosed space that goes around both ends
% 3) There are 3+ points - in this case, the uniqueStartPoints are assumed
%    to make an enclosed space and these are used.
if Npoints==1
    preExpansionPoints  = uniqueStartPoints(1,:) + preExpansionRadius*[cos(angles) sin(angles)];
elseif Npoints==2 
    directionVector1to2 = uniqueStartPoints(2,:)-uniqueStartPoints(1,:);
    lineAngle = atan2(directionVector1to2(2),directionVector1to2(1));
    positiveOrNegative = sin(angles);
    positiveAngles = angles(positiveOrNegative>=0);
    nPositive = sum(positiveOrNegative>=0);

    preExpansionPoints = [...
        ones(nPositive,1)*uniqueStartPoints(2,:) + preExpansionRadius*[cos(positiveAngles+lineAngle-pi/2) sin(positiveAngles+lineAngle-pi/2)];
        ones(nPositive,1)*uniqueStartPoints(1,:) + preExpansionRadius*[cos(positiveAngles+lineAngle+pi/2) sin(positiveAngles+lineAngle+pi/2)];
        ];
    preExpansionPoints = [preExpansionPoints; preExpansionPoints(1,:)];

else
    %%%%
    % Points are already an enclosed space
    preExpansionPoints  = uniqueStartPoints;
end
end 