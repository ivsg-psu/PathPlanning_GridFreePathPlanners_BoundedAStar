% script_demo_BoundaryXp.m
% This script sets up dependencies for the BoundaryXp library and runs
% demos showing the library's capabilities.
%
% REVISION HISTORY:
% 2026_02_27 - K. Hayes
% - In this demo script
%       % * created demo script using script_demo_BoundedAStar as starter
% - DEPRECATED:
%       % * fcn_BoundedAStar_reachabilityWithInputs_SLOW
%       % * script_test_fcn_BoundedAStar_reachabilityWithInputs_SLOW
%       % * fcn_BoundedAStar_calcCostChangingWind
%       % * script_test_fcn_BoundedAStar_calcCostChangingWind
%       % * fcn_BoundedAStar_calcCostConstantWind
%       % * script_test_fcn_BoundedAStar_calcCostConstantWind
%       % * fcn_BoundedAStar_calcControlInput
%       % * script_test_fcn_BoundedAStar_calcControlInput
%       % * fcn_BoundedAStar_matrixEnvelopeExpansion
%       % * script_test_fcn_BoundedAStar_matrixEnvelopeExpansion
%       % * fcn_BoundedAStar_pathCalculation
%       % * script_test_fcn_BoundedAStar_pathCalculation
%       % * fcn_BoundedAStar_pathCalculationToStart
%       % * script_test_fcn_BoundedAStar_pathCalculationToStart
%       % * fcn_BoundedAStar_rrtWindGraph
%       % * fcn_BoundedAStar_generateWindGraph
%       % * script_test_fcn_BoundedAStar_generateWindGraph
%       % * fcn_BoundedAStar_fillWindField
%       % * script_test_fcn_BoundedAStar_fillWindField
%
% 2026_04_01 - K. Hayes
% - Imported BoundaryXP functions developed in private repository
% - ADDED:
%       % * fcn_BoundaryXP_expandVerticesOutward
%       % * fcn_BoundaryXP_prepareStartPoints
%       % * fcn_BoundaryXP_processExpandedPoints
%       % * fcn_BoundaryXP_resamplePointsToMatchMapDiscretization
% - In fcn_BoundedAStar_reachabilityWithInputs
%       % * replaced internal functions with separated versions
% - ADDED: 
%       % * fcn_BoundaryXP_makeRegion
%       % * fcn_BoundaryXP_checkBoundaries


%
% TO DO:
% - revise dependencies when BoundedAStar is finished as a release

%% Set up path and dependencies
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%    
%    _____      _               
%  / ____|    | |              
% | (___   ___| |_ _   _ _ __  
%  \___ \ / _ \ __| | | | '_ \ 
%  ____) |  __/ |_| |_| | |_) |
% |_____/ \___|\__|\__,_| .__/ 
%                       | |    
%                       |_|                                                  
%
% See: https://patorjk.com/software/taag/#p=display&f=Big&t=Setup
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Make sure we are running out of root directory
st = dbstack; 
thisFile = which(st(1).file);
[filepath,name,ext] = fileparts(thisFile);
cd(filepath);

%% Clear paths and folders if needed
if 1==1
    clear flag_BoundaryXp_Folders_Initialized
end
if 1==0
    fcn_INTERNAL_clearUtilitiesFromPathAndFolders;
end

%% Install dependencies
% Define a universal resource locator (URL) pointing to the repos of
% dependencies to install. Note that DebugTools is always installed
% automatically, first, even if not listed:
clear dependencyURLs dependencySubfolders
ith_repo = 0;

ith_repo = ith_repo+1;
dependencyURLs{ith_repo} = 'https://github.com/ivsg-psu/PathPlanning_MapTools_MapGenClassLibrary';
dependencySubfolders{ith_repo} = {'Functions','testFixtures','GridMapGen'};

ith_repo = ith_repo+1;
dependencyURLs{ith_repo} = 'https://github.com/ivsg-psu/PathPlanning_GridFreePathPlanners_VGraph';
dependencySubfolders{ith_repo} = {'Functions','Data'};

ith_repo = ith_repo+1;
dependencyURLs{ith_repo} = 'https://github.com/ivsg-psu/PathPlanning_PathTools_PathClassLibrary';
dependencySubfolders{ith_repo} = {'Functions','Data'};

ith_repo = ith_repo+1;
dependencyURLs{ith_repo} = 'https://github.com/ivsg-psu/FieldDataCollection_VisualizingFieldData_PlotRoad';
dependencySubfolders{ith_repo} = {'Functions','Data'};

ith_repo = ith_repo+1;
dependencyURLs{ith_repo} = 'https://github.com/ivsg-psu/PathPlanning_GeomTools_GeomClassLibrary';
dependencySubfolders{ith_repo} = {'Functions','Data'};

%% Do we need to set up the work space?
if ~exist('flag_BoundaryXp_Folders_Initialized','var')

    % Clear prior global variable flags
    clear global FLAG_*

    % Navigate to the Installer directory
    currentFolder = pwd;
    cd('Installer');
    % Create a function handle
    func_handle = @fcn_DebugTools_autoInstallRepos;

    % Return to the original directory
    cd(currentFolder);

    % Call the function to do the install
    func_handle(dependencyURLs, dependencySubfolders, (0), (-1));

    % Add this function's folders to the path
    this_project_folders = {...
        'Functions',...
        'Data',...
        'Functions_CostCalculation',...
        'Test_Fixtures',...
        'Utilities_gif',...
        'Utilities_TriangleRayIntersection',...
        'Utilities_streamcolor'};
    fcn_DebugTools_addSubdirectoriesToPath(pwd,this_project_folders)

    flag_BoundAStar_Folders_Initialized = 1;
end

%% Set environment flags for input checking in BoundedAStar library
% These are values to set if we want to check inputs or do debugging
setenv('MATLABFLAG_BOUNDARYXP_FLAG_CHECK_INPUTS','1');
setenv('MATLABFLAG_BOUNDARYXP_FLAG_DO_DEBUG','0');

%% Finish setup, show demos
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 
%    _____      _   _   _                _____ _             _           _ 
%   / ____|    | | | | (_)              / ____| |           | |         | |
%  | |  __  ___| |_| |_ _ _ __   __ _  | (___ | |_ __ _ _ __| |_ ___  __| |
%  | | |_ |/ _ \ __| __| | '_ \ / _` |  \___ \| __/ _` | '__| __/ _ \/ _` |
%  | |__| |  __/ |_| |_| | | | | (_| |  ____) | || (_| | |  | ||  __/ (_| |
%   \_____|\___|\__|\__|_|_| |_|\__, | |_____/ \__\__,_|_|   \__\___|\__,_|
%                                __/ |                                     
%                               |___/                                      
% See http://patorjk.com/software/taag/#p=display&f=Big&t=Getting%20Started
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

disp('Welcome to the BoundaryXp library!')

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

%% function fcn_INTERNAL_clearUtilitiesFromPathAndFolders
function fcn_INTERNAL_clearUtilitiesFromPathAndFolders
% Clear out the variables
clear global flag* FLAG*
clear flag*
clear path

% Clear out any path directories under Utilities
path_dirs = regexp(path,'[;]','split');
utilities_dir = fullfile(pwd,filesep,'Utilities');
for ith_dir = 1:length(path_dirs)
    utility_flag = strfind(path_dirs{ith_dir},utilities_dir);
    if ~isempty(utility_flag)
        rmpath(path_dirs{ith_dir});
    end
end

% Delete the Utilities folder, to be extra clean!
if  exist(utilities_dir,'dir')
    [status,message,message_ID] = rmdir(utilities_dir,'s');
    if 0==status
        error('Unable remove directory: %s \nReason message: %s \nand message_ID: %s\n',utilities_dir, message,message_ID);
    end
end

end % Ends fcn_INTERNAL_clearUtilitiesFromPathAndFolders