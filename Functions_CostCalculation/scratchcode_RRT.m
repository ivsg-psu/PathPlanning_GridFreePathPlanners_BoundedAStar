% Basic RRT implementation

% Define needed variables
start = [-5 10];
finish = [9 4];
%%% in recursive implementation to find the paths between different nodes
%%% based on the wind field, start and finish should be iteratively changed
%%% to match the 'start' and 'goal' node for that specific edge
XY_range = [-10 -10 10 10];

% Assign wind fields from other script run
windFieldU = normalizedEastWind;
windFieldV = normalizedNorthWind;

% Set RRT* parameters
maxIter = 10000;
stepSize = 1;
goalRadius = 0.5;
rng(6655)

% Initialize storage variables
vertices = [start, 0];
edges = nan*ones(maxIter);

% Create figure for debug
figure
hold on
axis equal
s = streamslice(x,y,windFieldU,windFieldV);
set(s, 'Color', [0.6 0.6 0.6]);
plot(start(1),start(2),'rx','MarkerSize',20,'Linewidth',3)
plot(finish(1),finish(2),'gx','MarkerSize',20,'LineWidth',3)


% Do RRT
for i = 1:maxIter
    % Find a random position
    % Fix adjustment later
    pRand = 20*rand(1,2)-10;

    % Find nearest vertex
    pRandVec = pRand.*ones(size(vertices));
    dist = sum([pRandVec - vertices].^2,2).^0.5;
    minInd = find(dist == min(dist));
    pNearest = vertices(minInd,:);

    % New point using random position, nearest vertex, stepSize
    heading = [pRand - pNearest]/norm(pRand - pNearest);

    xIndex = find(x>pRand(1),1,'first');
    yIndex = find(y>pRand(2),1,'first');
    if isempty(xIndex)
        xIndex = find(x<pRand(1),1,'first');
    end
    if isempty(yIndex)
        yIndex = find(y<pRand(2),1,'first');
    end
    
    % Convert indices into linear indices for indexing wind fields
    linearInd = sub2ind(size(windFieldU),yIndex,xIndex);

    Wu = windFieldU(linearInd);
    Wv = windFieldV(linearInd);
    W = [Wu Wv];

    step = (heading + W)*stepSize;
    adjustment = stepSize./norm(step);
    newStep = step.*adjustment;

    pNew = pNearest + newStep;

    % Add the new vertex to the graph
    parentNode = find(vertices)
    vertices = [vertices; pNew];

    % Add new vertex to graph
    plot([pNew(1),pNearest(1)],[pNew(2),pNearest(2)],'LineWidth',2,'Color','black')
    drawnow

    % Check if goal reached
    if norm(pNew - finish) <= goalRadius
        break
    end
end