% script_test_fcn_BoundedAStar_reachabilityWithInputs
% Tests: fcn_BoundedAStar_reachabilityWithInputs

% Revision history
% 2025_07_29 by S. Brennan, sbrennan@psu.edu
% - first write of script 
%   % * using script_test_fcn_BoundedAStar_matrixEnvelopeExpansion as 
%   %   % starter
%
% 2025_08_02 by S. Brennan, sbrennan@psu.edu
% - In script_test_fcn_BoundedAStar_reachabilityWithInputs
%   % Added BUG case 90001
%   % * illustrates: expansion glitching observed due to bug in Path library 
%   % * Now fixed with Path library updates
%   % Added BUG case 90002
%   % * expansion produces NaN values
%   % Added BUG case 90003
%   % * input startPoints have self cross over
%
% 2025_08_08 by S. Brennan, sbrennan@psu.edu
% - In script_test_fcn_BoundedAStar_reachabilityWithInputs
%   % * Added test scripts to check new output: boundingPolytopeVertices

% TO DO:
% -- take wind post-processing out of this script and put it into another
%    separate function


%% Set up the workspace
close all

%% Code demos start here
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%   _____                              ____   __    _____          _
%  |  __ \                            / __ \ / _|  / ____|        | |
%  | |  | | ___ _ __ ___   ___  ___  | |  | | |_  | |     ___   __| | ___
%  | |  | |/ _ \ '_ ` _ \ / _ \/ __| | |  | |  _| | |    / _ \ / _` |/ _ \
%  | |__| |  __/ | | | | | (_) \__ \ | |__| | |   | |___| (_) | (_| |  __/
%  |_____/ \___|_| |_| |_|\___/|___/  \____/|_|    \_____\___/ \__,_|\___|
%
%
% See: https://patorjk.com/software/taag/#p=display&f=Big&t=Demos%20Of%20Code
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Figures start with 1

close all;
fprintf(1,'Figure: 1XXXXXX: DEMO cases\n');

%% DEMO case: get reachable envelope in wind field (2 starting points)
figNum = 10001;
titleString = sprintf('DEMO case: get reachable envelope in wind field (2 starting points)');
fprintf(1,'Figure %.0f: %s\n',figNum, titleString);
figure(figNum); clf;

% Load starting data
[normalizedEastWind, normalizedNorthWind, windFieldX, windFieldY] = fcn_INTERNAL_loadExampleData;

% Call graph generation function
radius = 2;
maxWindSpeed = 4;

windFieldU = normalizedEastWind*maxWindSpeed;
windFieldV = normalizedNorthWind*maxWindSpeed;
startPoints = [0 0; -1 -2];
flagWindRoundingType = 0;

% Call function
[reachableSet, boundingPolytopeVertices] =  fcn_BoundedAStar_reachabilityWithInputs(...
    radius, windFieldU, windFieldV, windFieldX, windFieldY, (startPoints), (flagWindRoundingType), (figNum));

sgtitle(titleString, 'Interpreter','none');

% Check variable types
assert(isnumeric(reachableSet));
assert(isnumeric(boundingPolytopeVertices));

% Check variable sizes
assert(size(reachableSet,1)>=3); 
assert(size(reachableSet,2)==2);
assert(size(boundingPolytopeVertices,1)>=3); 
assert(size(boundingPolytopeVertices,2)==2);

% Make sure plot opened up
assert(isequal(get(gcf,'Number'),figNum));

%% DEMO case: get reachable envelope in wind field (1 starting point)
figNum = 10002;
titleString = sprintf('DEMO case: get reachable envelope in wind field (1 starting points)');
fprintf(1,'Figure %.0f: %s\n',figNum, titleString);
figure(figNum); clf;

% Load starting data
[normalizedEastWind, normalizedNorthWind, windFieldX, windFieldY] = fcn_INTERNAL_loadExampleData;

% Call graph generation function
radius = 2;
maxWindSpeed = 4;

windFieldU = normalizedEastWind*maxWindSpeed;
windFieldV = normalizedNorthWind*maxWindSpeed;
startPoints = [0 0];
flagWindRoundingType = 0;

% Call function
[reachableSet, boundingPolytopeVertices] =  fcn_BoundedAStar_reachabilityWithInputs(...
    radius, windFieldU, windFieldV, windFieldX, windFieldY, (startPoints), (flagWindRoundingType), (figNum));

sgtitle(titleString, 'Interpreter','none');

% Check variable types
assert(isnumeric(reachableSet));
assert(isnumeric(boundingPolytopeVertices));

% Check variable sizes
assert(size(reachableSet,1)>=3); 
assert(size(reachableSet,2)==2);
assert(size(boundingPolytopeVertices,1)>=3); 
assert(size(boundingPolytopeVertices,2)==2);

% Make sure plot opened up
assert(isequal(get(gcf,'Number'),figNum));

%% DEMO case: get reachable envelope in wind field (square starting set)
figNum = 10003;
titleString = sprintf('DEMO case: get reachable envelope in wind field (square starting set)');
fprintf(1,'Figure %.0f: %s\n',figNum, titleString);
figure(figNum); clf;

% Load starting data
[normalizedEastWind, normalizedNorthWind, windFieldX, windFieldY] = fcn_INTERNAL_loadExampleData;

% Call graph generation function
radius = 2;
maxWindSpeed = 4;

windFieldU = normalizedEastWind*maxWindSpeed;
windFieldV = normalizedNorthWind*maxWindSpeed;
startPoints = [-1 -1; 1 -1; 1 1; -1 1; -1 -1];
flagWindRoundingType = 1;

% Call function
[reachableSet, boundingPolytopeVertices] =  fcn_BoundedAStar_reachabilityWithInputs(...
    radius, windFieldU, windFieldV, windFieldX, windFieldY, (startPoints), (flagWindRoundingType), (figNum));

sgtitle(titleString, 'Interpreter','none');

% Check variable types
assert(isnumeric(reachableSet));
assert(isnumeric(boundingPolytopeVertices));

% Check variable sizes
assert(size(reachableSet,1)>=3); 
assert(size(reachableSet,2)==2);
assert(size(boundingPolytopeVertices,1)>=3); 
assert(size(boundingPolytopeVertices,2)==2);

% Make sure plot opened up
assert(isequal(get(gcf,'Number'),figNum));

%% DEMO case: get reachable envelope in wind field (weird shape starting set)
figNum = 10003;
titleString = sprintf('DEMO case: get reachable envelope in wind field (square starting set)');
fprintf(1,'Figure %.0f: %s\n',figNum, titleString);
figure(figNum); clf;

% Load starting data
[normalizedEastWind, normalizedNorthWind, windFieldX, windFieldY] = fcn_INTERNAL_loadExampleData;

% Call graph generation function
radius = 0.1;
maxWindSpeed = 4;

windFieldU = normalizedEastWind*maxWindSpeed;
windFieldV = normalizedNorthWind*maxWindSpeed;
startPoints = [-1 -1; 1 -1; 1 1; -2 3; -1 1; -1 -1];
flagWindRoundingType = 0;

% Call function
[reachableSet, boundingPolytopeVertices] =  fcn_BoundedAStar_reachabilityWithInputs(...
    radius, windFieldU, windFieldV, windFieldX, windFieldY, (startPoints), (flagWindRoundingType), (figNum));

sgtitle(titleString, 'Interpreter','none');

% Check variable types
assert(isnumeric(reachableSet));
assert(isnumeric(boundingPolytopeVertices));

% Check variable sizes
assert(size(reachableSet,1)>=3); 
assert(size(reachableSet,2)==2);
assert(size(boundingPolytopeVertices,1)>=3); 
assert(size(boundingPolytopeVertices,2)==2);

% Make sure plot opened up
assert(isequal(get(gcf,'Number'),figNum));

%% Test cases start here. These are very simple, usually trivial
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%  _______ ______  _____ _______ _____
% |__   __|  ____|/ ____|__   __/ ____|
%    | |  | |__  | (___    | | | (___
%    | |  |  __|  \___ \   | |  \___ \
%    | |  | |____ ____) |  | |  ____) |
%    |_|  |______|_____/   |_| |_____/
%
%
%
% See: https://patorjk.com/software/taag/#p=display&f=Big&t=TESTS
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Figures start with 2

close all;
fprintf(1,'Figure: 2XXXXXX: TEST mode cases\n');

%% TEST case: Drop a point on a streamline
figNum = 20001;
titleString = sprintf('TEST case: Drop a point on a streamline');
fprintf(1,'Figure %.0f: %s\n',figNum, titleString);
figure(figNum); clf;


%% Fast Mode Tests
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%  ______        _     __  __           _        _______        _
% |  ____|      | |   |  \/  |         | |      |__   __|      | |
% | |__ __ _ ___| |_  | \  / | ___   __| | ___     | | ___  ___| |_ ___
% |  __/ _` / __| __| | |\/| |/ _ \ / _` |/ _ \    | |/ _ \/ __| __/ __|
% | | | (_| \__ \ |_  | |  | | (_) | (_| |  __/    | |  __/\__ \ |_\__ \
% |_|  \__,_|___/\__| |_|  |_|\___/ \__,_|\___|    |_|\___||___/\__|___/
%
%
% See: http://patorjk.com/software/taag/#p=display&f=Big&t=Fast%20Mode%20Tests
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Figures start with 8

close all;
fprintf(1,'Figure: 8XXXXXX: FAST mode cases\n');

%% Basic example - NO FIGURE
figNum = 80001;
fprintf(1,'Figure: %.0f: FAST mode, empty figNum\n',figNum);
figure(figNum); close(figNum);

% Load starting data
[normalizedEastWind, normalizedNorthWind, windFieldX, windFieldY] = fcn_INTERNAL_loadExampleData;

% Call graph generation function
radius = 2;
maxWindSpeed = 4;

windFieldU = normalizedEastWind*maxWindSpeed;
windFieldV = normalizedNorthWind*maxWindSpeed;
startPoints = [0 0; 1 2];
flagWindRoundingType = 0;

% Call function
[reachableSet, boundingPolytopeVertices] =  fcn_BoundedAStar_reachabilityWithInputs(...
    radius, windFieldU, windFieldV, windFieldX, windFieldY, (startPoints), (flagWindRoundingType), ([]));

% Check variable types
assert(isnumeric(reachableSet));
assert(isnumeric(boundingPolytopeVertices));

% Check variable sizes
assert(size(reachableSet,1)>=3); 
assert(size(reachableSet,2)==2);
assert(size(boundingPolytopeVertices,1)>=3); 
assert(size(boundingPolytopeVertices,2)==2);

% Make sure plot did NOT open up
figHandles = get(groot, 'Children');
assert(~any(figHandles==figNum));


%% Basic fast mode - NO FIGURE, FAST MODE
figNum = 80002;
fprintf(1,'Figure: %.0f: FAST mode, figNum=-1\n',figNum);
figure(figNum); close(figNum);

% Load starting data
[normalizedEastWind, normalizedNorthWind, windFieldX, windFieldY] = fcn_INTERNAL_loadExampleData;

% Call graph generation function
radius = 2;
maxWindSpeed = 4;

windFieldU = normalizedEastWind*maxWindSpeed;
windFieldV = normalizedNorthWind*maxWindSpeed;
startPoints = [0 0; 1 2];
flagWindRoundingType = 0;

% Call function
[reachableSet, boundingPolytopeVertices] =  fcn_BoundedAStar_reachabilityWithInputs(...
    radius, windFieldU, windFieldV, windFieldX, windFieldY, (startPoints), (flagWindRoundingType), (-1));

% Check variable types
assert(isnumeric(reachableSet));
assert(isnumeric(boundingPolytopeVertices));

% Check variable sizes
assert(size(reachableSet,1)>=3); 
assert(size(reachableSet,2)==2);
assert(size(boundingPolytopeVertices,1)>=3); 
assert(size(boundingPolytopeVertices,2)==2);

% Make sure plot did NOT open up
figHandles = get(groot, 'Children');
assert(~any(figHandles==figNum));


%% Compare speeds of pre-calculation versus post-calculation versus a fast variant
figNum = 80003;
fprintf(1,'Figure: %.0f: FAST mode comparisons\n',figNum);
figure(figNum);
close(figNum);

% Load starting data
[normalizedEastWind, normalizedNorthWind, windFieldX, windFieldY] = fcn_INTERNAL_loadExampleData;

% Call graph generation function
radius = 2;
maxWindSpeed = 4;

windFieldU = normalizedEastWind*maxWindSpeed;
windFieldV = normalizedNorthWind*maxWindSpeed;
startPoints = [0 0; 1 2];
flagWindRoundingType = 0;

Niterations = 10;

% Slow mode
tic;
for ith_test = 1:Niterations
    % Call function
    [reachableSet, boundingPolytopeVertices] =  fcn_BoundedAStar_reachabilityWithInputs(...
        radius, windFieldU, windFieldV, windFieldX, windFieldY, (startPoints), (flagWindRoundingType), ([]));
end
slow_method = toc;

% Check variable types
assert(isnumeric(reachableSet));
assert(isnumeric(boundingPolytopeVertices));

% Check variable sizes
assert(size(reachableSet,1)>=3); 
assert(size(reachableSet,2)==2);
assert(size(boundingPolytopeVertices,1)>=3); 
assert(size(boundingPolytopeVertices,2)==2);

% Do calculation with pre-calculation, FAST_MODE on
tic;
for ith_test = 1:Niterations
    % Call function
    [reachableSet, boundingPolytopeVertices] =  fcn_BoundedAStar_reachabilityWithInputs(...
        radius, windFieldU, windFieldV, windFieldX, windFieldY, (startPoints), (flagWindRoundingType), (-1));
end
fast_method = toc;

% Check variable types
assert(isnumeric(reachableSet));
assert(isnumeric(boundingPolytopeVertices));

% Check variable sizes
assert(size(reachableSet,1)>=3); 
assert(size(reachableSet,2)==2);
assert(size(boundingPolytopeVertices,1)>=3); 
assert(size(boundingPolytopeVertices,2)==2);

% Make sure plot did NOT open up
figHandles = get(groot, 'Children');
assert(~any(figHandles==figNum));

% Plot results as bar chart
figure(373737);
clf;
hold on;

X = categorical({'Normal mode','Fast mode'});
X = reordercats(X,{'Normal mode','Fast mode'}); % Forces bars to appear in this exact order, not alphabetized
Y = [slow_method fast_method ]*1000/Niterations;
bar(X,Y)
ylabel('Execution time (Milliseconds)')


% Make sure plot did NOT open up
figHandles = get(groot, 'Children');
assert(~any(figHandles==figNum));


%% BUG cases
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%  ____  _    _  _____
% |  _ \| |  | |/ ____|
% | |_) | |  | | |  __    ___ __ _ ___  ___  ___
% |  _ <| |  | | | |_ |  / __/ _` / __|/ _ \/ __|
% | |_) | |__| | |__| | | (_| (_| \__ \  __/\__ \
% |____/ \____/ \_____|  \___\__,_|___/\___||___/
%
% See: http://patorjk.com/software/taag/#p=display&v=0&f=Big&t=BUG%20cases
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% All bug case figures start with the number 9

% close all;

%% BUG case: expansion glitching observed due to bug in Path library (fixed)

figNum = 90001;
titleString = sprintf('BUG case: expansion glitching observed due to bug in Path library (fixed)');
fprintf(1,'Figure %.0f: %s\n',figNum, titleString);
figure(figNum); clf;

% Load starting data
[normalizedEastWind, normalizedNorthWind, windFieldX, windFieldY] = fcn_INTERNAL_loadExampleData;

% Call graph generation function
radius = 0.5;
maxWindSpeed = 1;

windFieldU = normalizedEastWind*maxWindSpeed;
windFieldV = normalizedNorthWind*maxWindSpeed;
flagWindRoundingType = 1;

startPoints = [...
   2.382127025781378  -1.581756646038485
   2.309171928057874  -1.145750021505348
   2.282662767131663  -0.995056722022869
   2.287367624690293  -0.778032815426177
   2.247744261413241  -0.525867327852700
   2.208279641368899  -0.206304726305456
   2.145775523032383   0.118860304258495
   2.081955302838964   0.549011789156652
   2.078994206962031   0.567204558880521
   2.078933740172697   0.568964862575723
   2.076723186980467   0.794802658085192
   2.037745080745749   0.946168401815517
   2.027127603837225   1.254506866856157
   1.989338390626548   1.497173562918755
   2.001895008702639   1.641780062721229
   1.930135513814969   2.063220203428516
   1.932244116790294   2.211620463493408
   1.921302695557221   2.310029363908730
   1.916529472111721   2.355556066469140
   1.895158394421256   2.766151335663927
   1.800743553804144   3.058277285561736
   1.774065498775499   3.204378149659556
   1.738390081410453   3.624616415861778
   1.736715683315172   3.658424239320890
   1.738251001646683   3.703765529346810
   1.710917908567398   3.849388157275846
   1.643677175069175   4.262835799113564
   1.639326954526304   4.284805772741763
   1.628631663785328   4.399983417777301
   1.509471946209048   4.857135697111977
   1.491750965937543   4.904229031383296
   1.447182475622905   5.010201911953916
   1.377562699750000   5.301357478442928
   1.340499137030754   5.360135238133777
   1.263499783415980   5.647482315238316
   1.208577146194354   5.855095468604810
   1.179318205599123   5.981170693175845
   1.143385888457039   6.241678027040651
   1.068500574987591   6.647400820530041
   1.059910652824073   6.685259126668516
   1.057073362253109   6.692750706013676
   1.033053970738646   6.789179516398365
   0.991625343454381   6.898320655866334
   0.994974170539389   7.050456485628965
   0.843086642727983   7.485930215432880
   0.833732091940116   7.629193204207151
   0.781665534357850   7.718449009684743
   0.769635889385288   7.793001820512802
   0.648630144336804   8.182978125966031
   0.629682460540445   8.236232594995975
   0.613840503861417   8.320461017435294
   0.574497916559255   8.409176243869791
   0.529618299573746   8.481536623457213
   0.450912489717096   8.927496809084737
   0.383935274254919   9.111325249746969
   0.357805600339796   9.150294128399908
   0.242931587365623   9.465379059076781
   0.229687803219044   9.493431247348701
   0.134087767346208   9.890514023209640
   0.071147446804269  10.000000000000000
   0.091057580203941  10.000000000000000
  -0.047789575221813  10.000000000000000
  -0.259745868263779  10.000000000000000
  -0.370553034112621  10.000000000000000
  -0.658240694876143  10.000000000000000
  -0.669463430506475  10.000000000000000
  -0.859277180835856  10.000000000000000
  -0.892023047792607  10.000000000000000
  -1.149465154578340  10.000000000000000
  -1.251040191691276  10.000000000000000
  -1.656726309719013  10.000000000000000
  -1.833139380585729  10.000000000000000
  -2.013625078144860  10.000000000000000
  -2.014760834050346  10.000000000000000
  -2.170957689189497  10.000000000000000
  -2.330972248301397  10.000000000000000
  -2.710601891802636  10.000000000000000
  -2.992748221804290  10.000000000000000
  -3.020954585608771  10.000000000000000
  -3.197003387642314  10.000000000000000
  -3.235423364320732  10.000000000000000
  -3.235757599964169  10.000000000000000
  -3.643782171789784  10.000000000000000
  -3.692488775570480  10.000000000000000
  -3.902054052470677  10.000000000000000
  -3.941949540193592  10.000000000000000
  -4.078115640214073  10.000000000000000
  -4.083859515996343  10.000000000000000
  -4.162767393736051  10.000000000000000
  -4.324224185901409   9.824204927805750
  -4.325880395739054   9.820062647102835
  -4.432784176320409   9.599120942443028
  -4.597640105368888   9.222377458826704
  -4.602683083428422   9.210164973474050
  -4.612489276395815   9.185170823632577
  -4.615083444945378   9.178552386504375
  -4.626614021409593   9.148756269663032
  -4.781423335372860   8.747382452112625
  -4.810973304679529   8.672431254299688
  -4.947993451479954   8.286427216964935
  -4.959102314083293   8.253145858738813
  -4.965045410705735   8.235550786290254
  -4.972009799789523   8.214445469432942
  -4.972227178981768   8.213788728541733
  -5.109066318858013   7.828075050062203
  -5.109944750386238   7.825461244932232
  -5.119394726265670   7.796340238247943
  -5.230450581869198   7.371238730203682
  -5.234703181133832   7.355093003656838
  -5.237672886395666   7.343446789410438
  -5.281746177721658   7.208075515121629
  -5.351014779112636   6.807068765375808
  -5.359553112519252   6.755667885722215
  -5.366487306530830   6.715809300534199
  -5.368248958765422   6.700955852990323
  -5.372466700692971   6.679708032518400
  -5.385746731599895   6.586751705643033
  -5.401494744096520   6.550169464730833
  -5.458886892445668   6.147405351376388
  -5.467925106341032   6.083264619606087
  -5.474651158863979   6.027961491566562
  -5.480537105230536   5.993080497838536
  -5.494957661047779   5.920652273246504
  -5.528868932107232   5.514900154668452
  -5.530973404663182   5.492396407229674
  -5.535001498446442   5.455292717972746
  -5.563743333585643   5.053228312349077
  -5.592992191748974   4.632324361636435
  -5.624714023239346   4.237841961385373
  -5.439525900119217   3.893751887645100
  -5.379570499640568   3.764138562841199
  -5.292513596007574   3.597390056874884
  -5.192967773591474   3.411700394532549
  -5.122815474759375   3.269336729033403
  -5.008209796913376   3.072346026374110
  -4.893868849143731   2.866458210008825
  -4.721555659377401   2.492591187628865
  -4.531866019016983   2.124567702893874
  -4.520893642630461   2.101523662772469
  -4.501774238124407   2.059945187150757
  -4.462873870486730   1.976716786684448
  -4.423539650550629   1.935787610368081
  -4.406598936603188   1.900383874413231
  -4.355137300920402   1.782931943977208
  -4.202255115087323   1.473125816153161
  -4.199730851980305   1.285839075126465
  -4.000276368004180   0.905426214094758
  -3.996662089808289   0.898472669387540
  -3.838961004441859   0.619592854696690
  -3.736413873521820   0.381280869678112
  -3.723047833796670   0.187147708694605
  -3.516875080173192  -0.201202124065181
  -3.439482640910744  -0.357332257127539
  -3.354916144893233  -0.517268436158132
  -3.268818668096858  -0.765079997863634
  -3.086543886950693  -1.114559700263334
  -3.046634774406918  -1.257583726737489
  -2.994326243340959  -1.411383551271115
  -2.878513161246139  -1.707314616711166
  -2.777421144947089  -2.166422374542876
  -2.765874560939967  -2.206323457311877
  -2.755129658279582  -2.234217835743047
  -2.754286240495925  -2.236467653329442
  -2.745504680173373  -2.518142093681397
  -2.667430203472927  -2.953331623350579
  -2.658146217169409  -3.010513172798348
  -2.651084455894715  -3.059060547106817
  -2.602561992100308  -3.337377148605635
  -2.448621451024428  -3.715397731323853
  -2.458283482291985  -3.838740401338469
  -2.430358025685290  -3.919683859554162
  -2.348871181571465  -4.243997441669140
  -2.334001846869088  -4.518860022107216
  -2.307086173048710  -4.959220578259852
  -2.304739239618573  -4.996134104401208
  -2.299604790935347  -5.139976864436457
  -2.207209803340356  -5.567047385826492
  -2.191308181815811  -5.631691411090072
  -2.144412481188230  -5.860932881630895
  -2.090648056755816  -6.034815762555885
  -2.081109112815798  -6.224935949403034
  -2.052563356153207  -6.356457365810311
  -2.044928038291765  -6.511684708787414
  -2.071157523700078  -6.737992326953096
  -2.070573965188356  -7.171919055273785
  -2.069606240591527  -7.205460378491222
  -2.100585720113691  -7.440513026796211
  -2.093546847374232  -7.832447477333912
  -2.020201963499193  -8.092229036309801
  -2.014954390715236  -8.109599412550326
  -1.965988605888858  -8.496101335825498
  -1.948962010360359  -8.916951766126401
  -1.948345626571796  -8.924022834859050
  -1.945031492521059  -8.947358798273743
  -1.944100384527854  -9.036062247214780
  -1.921093530579888  -9.347470931846175
  -1.969271602172046  -9.585458122458622
  -1.923394885028928  -9.993394957832987
  -1.912317451889930 -10.000000000000000
  -1.888189168968844 -10.000000000000000
  -1.883214120096429 -10.000000000000000
  -1.850137376523163 -10.000000000000000
  -1.842593740205858 -10.000000000000000
  -1.438150056073449 -10.000000000000000
  -1.282150569571618 -10.000000000000000
  -1.271275308298065 -10.000000000000000
  -1.230386410420707 -10.000000000000000
  -0.959776160069638 -10.000000000000000
  -0.935579568890773 -10.000000000000000
  -0.695674454731274 -10.000000000000000
  -0.508236261560299 -10.000000000000000
  -0.403724428034244 -10.000000000000000
  -0.113202580216866 -10.000000000000000
   0.009725508986301 -10.000000000000000
   0.132653598189468 -10.000000000000000
   0.239109164411178 -10.000000000000000
   0.362037253614345 -10.000000000000000
   0.484965342817512 -10.000000000000000
   0.640757718857143 -10.000000000000000
   0.695216122641762 -10.000000000000000
   0.749674526426381 -10.000000000000000
   0.804132930210999 -10.000000000000000
   0.858591333995618 -10.000000000000000
   0.909635296066742 -10.000000000000000
   1.305228628984685 -10.000000000000000
   1.444381615187966 -10.000000000000000
   1.447659155754255 -10.000000000000000
   1.481076815891520 -10.000000000000000
   1.482235820071811 -10.000000000000000
   1.881985159626964 -10.000000000000000
   1.969735542518854 -10.000000000000000
   1.977868889794281 -10.000000000000000
   1.984219329949916 -10.000000000000000
   1.994846415902377 -10.000000000000000
   2.398816324107323 -10.000000000000000
   2.490412180521793 -10.000000000000000
   2.492103818460295 -10.000000000000000
   2.493095197046361 -10.000000000000000
   2.502271672424356 -10.000000000000000
   2.470374504391684 -10.000000000000000
   2.446949372768313 -10.000000000000000
   2.446154997708351 -10.000000000000000
   2.444835594096666 -10.000000000000000
   2.443295666036538 -10.000000000000000
   2.456553299635617  -9.970726490539924
   2.432762135307580  -9.550209177507519
   2.432663642790701  -9.505224364917018
   2.432645876985606  -9.490192487766175
   2.445819581923398  -9.397015056813711
   2.421972708902739  -8.972590088685861
   2.421796238029470  -8.923864351569925
   2.403275359954259  -8.822352966103811
   2.388334168606825  -8.721441382434039
   2.385577163154860  -8.609487951398584
   2.384269123148222  -8.449426242191560
   2.335939528330120  -8.018664350595104
   2.313374798971051  -7.955233594029649
   2.312787216327352  -7.936943966163524
   2.312678821828013  -7.906093910804441
   2.269115393286328  -7.516018740856389
   2.254170667112141  -7.180903149704974
   2.259425538962556  -7.146641038231740
   2.229312861107409  -6.706205330868721
   2.229270548775036  -6.704755338487203
   2.203167601068126  -6.261904593182545
   2.186313221999078  -6.193787335129641
   2.178918245320136  -5.989584904506743
   2.156937109514849  -5.639530380298661
   2.178373783440816  -5.262706036812473
   2.171386925660058  -4.814101736766753
   2.169865720231044  -4.810353297630840
   2.192806029882418  -4.716275166069134
   2.165976639320939  -4.267409650248925
   2.165934021317590  -4.266237487242626
   2.165151800322425  -4.277421013239512
   2.163758041001169  -4.308816723501915
   2.196823763475846  -4.262140344901580
   2.173934819930056  -3.813061019013791
   2.167766020806340  -3.602311953073425
   2.147678692908261  -3.302972818002143
   2.148527215493233  -3.234557920629502
   2.188792093512808  -2.849216505700583
   2.230738648038003  -2.465935358644869
   2.286084265794200  -2.086802949841652
   2.348994292812441  -1.694120498321195
   2.350428253629638  -1.688661147598049
   2.382127025781378  -1.581756646038485];

% startPoints = fcn_Path_removePinchPointInPath
figure(987); clf;
startPoints = fcn_Path_cleanPathFromForwardBackwardJogs(startPoints, [], 987);
% startPoints = fcn_Path_removePinchPointInPath(startPoints,9873);

deltaX = windFieldX(2) - windFieldX(1);
startPoints = fcn_INTERNAL_sparsifyPoints(startPoints,deltaX);

% Call function
[reachableSet, boundingPolytopeVertices] =  fcn_BoundedAStar_reachabilityWithInputs(...
    radius, windFieldU, windFieldV, windFieldX, windFieldY, (startPoints), (flagWindRoundingType), (figNum));

% Check variable types
assert(isnumeric(reachableSet));
assert(isnumeric(boundingPolytopeVertices));

% Check variable sizes
assert(size(reachableSet,1)>=3); 
assert(size(reachableSet,2)==2);
assert(size(boundingPolytopeVertices,1)>=3); 
assert(size(boundingPolytopeVertices,2)==2);

%% BUG case: expansion produces NaN values

figNum = 90002;
titleString = sprintf('BUG case: expansion produces NaN values');
fprintf(1,'Figure %.0f: %s\n',figNum, titleString);
figure(figNum); clf;

% Load starting data
[normalizedEastWind, normalizedNorthWind, windFieldX, windFieldY] = fcn_INTERNAL_loadExampleData;

% Call graph generation function
radius = 0.5;
maxWindSpeed = 1;

windFieldU = normalizedEastWind*maxWindSpeed;
windFieldV = normalizedNorthWind*maxWindSpeed;
flagWindRoundingType = 1;

startPoints = [...
   -6.256652947871200   3.008257015566111
  -6.009288064777866   2.665495075734295
  -5.733316232143466   2.267337281164038
  -5.483359631588089   1.918483387564855
  -5.159957328323609   1.446137545781391
  -4.865020394500964   0.880455195013401
  -4.626497399569535   0.409956013860186
  -4.329702273724639  -0.204850772308483
  -4.011137905510061  -0.968745811743577
  -3.862499622814750  -1.404840002866588
  -3.700808935397514  -1.859372346772682
  -3.587844716947598  -2.275666633569848
  -3.460819239380262  -2.684187811853350
  -3.368712724951822  -3.113175964687169
  -3.248944564961259  -3.588172297442876
  -3.185374851499917  -4.000952848780294
  -3.088191573443779  -4.428595272765089
  -2.989531413501223  -5.057180239569683
  -2.874403504959835  -5.848755775127359
  -2.834375206019993  -6.393385543895389
  -2.789008280519794  -6.860490033742742
  -2.653106523032700  -7.484329776056643
  -2.522371435487043  -8.299962877502388
  -2.333945855175635  -8.952199922991822
  -2.125732177201455  -9.667443032102199
  -1.866772204298977 -10.000000000000000
  -1.721858790378801 -10.000000000000000
  -1.298267681503475 -10.000000000000000
  -0.894119977469031 -10.000000000000000
  -0.422235029574080 -10.000000000000000
   0.055454720387466 -10.000000000000000
   0.599431168316989 -10.000000000000000
   1.058674559610628 -10.000000000000000
   1.845319770810525 -10.000000000000000
   2.460779385470633 -10.000000000000000
   3.002309564345893 -10.000000000000000
   3.537204573675556 -10.000000000000000
   3.615655602443650 -10.000000000000000
   3.606492162982800 -10.000000000000000
   3.498986746194721  -9.678889776470488
   3.414921298778804  -9.160771181803405
   3.335886265944668  -8.655348998195610
   3.232977789699884  -7.903584463998968
   3.169189888682440  -7.399085237279460
   3.100249474837739  -6.882436643590914
   2.997262279406480  -6.089473485562081
   2.898690338590370  -5.240892086318910
   2.836370956969111  -4.577511443644955
   2.817676144749943  -3.950401946301995
   2.788140407214584  -3.430598573999487
   2.750716080760394  -2.884517906386122
   2.712608942041605  -2.281925555478245
   2.675361926500942  -1.569677874579223
   2.651783759507289  -0.811122286904971
   2.672646565928624  -0.383531475898738
   2.673418105017146   0.052594326314242
   2.678326726556932   0.868053376677483
   2.678596131448925   1.349204977517446
   2.679171380284811   1.891833286000406
   2.671469345529856   2.353857692456885
   2.678843106790323   2.816823037026191
   2.686012327649892   3.286741215812174
   2.694877280906030   3.863464608838262
   2.687047146826401   4.396510337154050
   2.697338417586937   4.950149580630411
   2.691360119299948   5.384043884658916
   2.695759203455343   5.920016501030542
   2.665291089720265   6.461950757749825
   2.616576157201778   6.945284428755727
   2.555636258389330   7.617484074337320
   2.484700434310358   8.346624963020517
   2.369990547588630   9.078075878172363
   2.258330805170693   9.671055363550961
   1.943138320571151  10.000000000000000
   1.326101504470653  10.000000000000000
   0.578483591504372  10.000000000000000
   0.082937763016286  10.000000000000000
  -0.400189587194260  10.000000000000000
  -0.958406312115089  10.000000000000000
  -1.653838983548432  10.000000000000000
  -2.255982294359304  10.000000000000000
  -2.891278519505004  10.000000000000000
  -3.466677916064189  10.000000000000000
  -4.053727965112309  10.000000000000000
  -4.522847780376436  10.000000000000000
  -4.931328478939419  10.000000000000000
  -5.470267565409471  10.000000000000000
  -6.058609598328752  10.000000000000000
  -6.467706065446508  10.000000000000000
  -6.980445416304732  10.000000000000000
  -7.435502879204648  10.000000000000000
  -7.597751567063585  10.000000000000000
  -7.850132287736398   9.419796041203677
  -8.085961714434454   8.603049626998850
  -8.182592816710093   8.205796973590408
  -8.252731097868152   7.738369731125626
  -8.302516790323416   7.277464694530751
  -8.271675415131032   6.555660419949108
  -8.112465815107580   5.820875815957066
  -7.816992264216865   5.112616478147681
  -7.586572473591508   4.747427474399207
  -7.179548909108190   4.185651069607671
  -6.861482025808527   3.685714792560558
  -6.576908413288876   3.390661912112713
  -6.256652947871200   3.008257015566111];

% Call function
[reachableSet, boundingPolytopeVertices] =  fcn_BoundedAStar_reachabilityWithInputs(...
    radius, windFieldU, windFieldV, windFieldX, windFieldY, (startPoints), (flagWindRoundingType), (figNum));

% Check variable types
assert(isnumeric(reachableSet));
assert(isnumeric(boundingPolytopeVertices));

% Check variable sizes
assert(size(reachableSet,1)>=3); 
assert(size(reachableSet,2)==2);
assert(size(boundingPolytopeVertices,1)>=3); 
assert(size(boundingPolytopeVertices,2)==2);


%% BUG case: startPoints have self cross over

figNum = 90003;
titleString = sprintf('BUG case: startPoints have self cross over');
fprintf(1,'Figure %.0f: %s\n',figNum, titleString);
figure(figNum); clf;

% Load starting data
[normalizedEastWind, normalizedNorthWind, windFieldX, windFieldY] = fcn_INTERNAL_loadExampleData;

% Call graph generation function
radius = 0.5;
maxWindSpeed = 1;

windFieldU = normalizedEastWind*maxWindSpeed;
windFieldV = normalizedNorthWind*maxWindSpeed;
flagWindRoundingType = 1;

load('BUG_90003_fcn_BoundedAStar_reachabilityWithInputs.mat','startPoints');

startPointsPinched = fcn_Path_removePinchPointInPath(startPoints(1:end-1,:),2347);
% If more than half the points show up in the "pinch", then the pinch is
% inverted. We want to keep the points NOT in the pinch. Use the "setdiff"
% command to do this.
if length(startPointsPinched(:,1))<length(startPoints(:,1)/2)
    startPoints = setdiff(startPoints,startPointsPinched,'rows','stable');
end

% Call function
[reachableSet, boundingPolytopeVertices] =  fcn_BoundedAStar_reachabilityWithInputs(...
    radius, windFieldU, windFieldV, windFieldX, windFieldY, (startPoints), (flagWindRoundingType), (figNum));

% Check variable types
assert(isnumeric(reachableSet));
assert(isnumeric(boundingPolytopeVertices));

% Check variable sizes
assert(size(reachableSet,1)>=3); 
assert(size(reachableSet,2)==2);
assert(size(boundingPolytopeVertices,1)>=3); 
assert(size(boundingPolytopeVertices,2)==2);

%% Fail conditions
if 1==0
    %
       
end


%% Functions follow
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   ______                _   _
%  |  ____|              | | (_)
%  | |__ _   _ _ __   ___| |_ _  ___  _ __  ___
%  |  __| | | | '_ \ / __| __| |/ _ \| '_ \/ __|
%  | |  | |_| | | | | (__| |_| | (_) | | | \__ \
%  |_|   \__,_|_| |_|\___|\__|_|\___/|_| |_|___/
%
% See: https://patorjk.com/software/taag/#p=display&f=Big&t=Functions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%§

%% fcn_INTERNAL_loadExampleData
function [normalizedEastWind, normalizedNorthWind, windFieldX, windFieldY] = fcn_INTERNAL_loadExampleData
% Fill inputs
% 1 produces left to right, with edge patches that have right to left
% 2 produces left to right on top half, and top to bottom on bottom half
% 3 produces left to right on left half, bottom to top on right half
% 4 produces a saddle point near the middle
% 4822262 produces wind field that is mostly left to right

randomSeed = 4;

% Call random occupancy map function - code taken from
% script_demo_generateRandomOccupancyAnimated
rng(randomSeed)
nRows = 50;
mColumns = 50;
mapSize = [nRows mColumns];

occupancyRatio = 0.2;
dilationLevel = 400;
seedMap = rand(nRows,mColumns);
leftDilationMultiplier = [];
rightDilationMultiplier = [];
optimizedThreshold = [];

% Call the function once to initialize settings for upcoming calls
% [occupancyMatrix, randomMatrixDilated, forcedThreshold, leftDilationMultiplier, rightDilationMultiplier] = ...
[~, randomMatrixDilated, ~, ~, ~] = ...
    fcn_GridMapGen_generateRandomOccupancyMap(...
    'mapSize', (mapSize),... % [nRows mCols])
    'occupancyRatio',(occupancyRatio),... % [1x1] value between 0 and 1
    'dilationLevel',(dilationLevel),.... % [1x1] strictly positive int
    'seedMap', (seedMap),... % [1x1] integer to be a random seed or NxM matrix of random numbers
    'leftDilationMultiplier', (leftDilationMultiplier),... %  [nRows nRows], ...
    'rightDilationMultiplier', (rightDilationMultiplier),... % [mCols mCols], ...
    'thresholdForced', (optimizedThreshold), ... % [1x1] scalar
    'flagSkipThresholdOptimization',(0),...% [1x1] scalar
    'figNum',(-1));


%%%% WIND POST-PROCESSING SHOULD BE MADE INTO AN ALTERNATE FUNCTION (?). FOR
%%%% NOW THE CODE IS JUST COPIED.
% Use the gradient to estimate wind direction
[px,py] = gradient(randomMatrixDilated);
eastWind  = py;
northWind = -px;

% Solve for the wind magnitude
windMagnitude = (eastWind.^2+northWind.^2).^0.5;
maxWind = max(windMagnitude,[],'all');
normalizedEastWind = eastWind./maxWind;
normalizedNorthWind = northWind./maxWind;

XY_range = [-10 -10 10 10];
windFieldX = linspace(XY_range(1), XY_range(3), nRows);
windFieldY = linspace(XY_range(2), XY_range(4), mColumns);

end % Ends fcn_INTERNAL_loadExampleData

%% fcn_INTERNAL_sparsifyPoints
function sparsePoints = fcn_INTERNAL_sparsifyPoints(densePoints,deltaX)

% Make sure first and last point are repeated, e.g. that the plot is a
% closed circuit
if ~isequal(densePoints(1,:),densePoints(end,:))
    densePoints(end,:) = densePoints(1,:);
end

segmentVectors = densePoints(2:end,:) - densePoints(1:end-1,:);
segmentLengths = sum(segmentVectors.^2,2).^0.5;

Npoints = length(densePoints(:,1));
currentPoint = 1;
currentDistance = 0;
sparsePointIndices = false(Npoints,1);
while currentPoint<Npoints
    currentPoint = currentPoint+1;
    currentDistance  = currentDistance + segmentLengths(currentPoint-1);

    % Did we "travel" farther than expected deltaX? If so, mark this point
    % so that it is kept.
    if currentDistance>=deltaX
        sparsePointIndices(currentPoint,1) = true;
        currentDistance = 0;
    end
end
% Turn last point on, as this is a repeat of the first
sparsePointIndices(end,1) = true;

sparsePoints = densePoints(sparsePointIndices,:);

if 1==0
    figure(388383);
    clf;
    plot(densePoints(:,1),densePoints(:,2),'.-','MarkerSize',30,'LineWidth',3,'DisplayName','Input: densePoints');
    hold on;
    axis equal
    plot(sparsePoints(:,1),sparsePoints(:,2),'.-','MarkerSize',10,'LineWidth',1,'DisplayName','Output: sparsePoints');
end


end % Ends fcn_INTERNAL_sparsifyPoints