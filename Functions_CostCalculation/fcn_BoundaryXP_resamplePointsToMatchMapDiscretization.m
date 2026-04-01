function resampledPolyPoints = fcn_BoundaryXP_resamplePointsToMatchMapDiscretization(inputPoints,stepSize)

% Make sure points circle back on each other. Otherwise, final edge never
% gets filled
if ~isequal(inputPoints(1,:),inputPoints(end,:))
    densePreExpansionPoints = [inputPoints; inputPoints(1,:)];
else
    densePreExpansionPoints = inputPoints;
end

edgeVectors = densePreExpansionPoints(2:end,:)-densePreExpansionPoints(1:end-1,:);
edgeLengths = sum(edgeVectors.^2,2).^0.5;
unitEdgeVectors = edgeVectors./edgeLengths;

resampledPolyPoints = [];
% For each vector, break vector up into stepSize chunks. If last part is
% "small", delete last point to keep stepSize chunk, or larger, in the
% segment.
for ith_edge = 1:length(unitEdgeVectors(:,1))
    thisStartPoint = densePreExpansionPoints(ith_edge,:);
    thisLength = edgeLengths(ith_edge);
    thisUnitVector = unitEdgeVectors(ith_edge,:);

    % Count how many stepSizes fit inside thisLength
    Nresamples = floor(thisLength/stepSize);

    % Make sure remainder is not "small". If it is, just reduce the cuts by
    % one so last cut is longer than others.
    remainingDistance = thisLength - Nresamples*stepSize;
    if remainingDistance < 0.5*stepSize && thisLength>stepSize
        Nresamples = Nresamples-1;
    end
    morePoints = (0:Nresamples)'*stepSize*thisUnitVector + thisStartPoint;
    resampledPolyPoints = [resampledPolyPoints; morePoints]; %#ok<AGROW>
end

% Circle back to first point
uniqueResampledPolyPoints = unique(resampledPolyPoints(1:end,:),'rows','stable');
resampledPolyPoints = [uniqueResampledPolyPoints; resampledPolyPoints(1,:)];
end % Ends fcn_BoundaryXP_resamplePointsToMatchMapDiscretization
