for i=1:10
    reachableSet = fcn_BoundedAStar_reachabilityWithInputs(...
        radius, windFieldU, windFieldV, windFieldX, windFieldY, (startPoints), (flagWindRoundingType), (-1));
end