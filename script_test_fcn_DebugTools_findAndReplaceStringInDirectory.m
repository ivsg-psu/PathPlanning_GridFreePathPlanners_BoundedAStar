% script_test_fcn_DebugTools_findAndReplaceStringInDirectory
% Searches over a source directory for filenames, and for each, creates a
% search query. Next, given a list of paths to check (directoryPaths),
% searches through these paths for the search query.

% Define the repo names
oldString = 'fcn_Visibility_clearAndBlockedPointsGlobal';
newString = 'fcn_BoundedAStar_clearAndBlockedPointsGlobal';

% Define the function inputs
directoryPaths = {...
    fullfile(cd);
    fullfile(cd,'Functions');
    fullfile(cd,'Functions','New_Functions');
    fullfile(cd,'Functions','DEPRECATED_TO_TIMECLEAN_UNUSED');
    fullfile(cd,'Functions','OLD_code');
 };
directoryQuery = [];

% sourceDirectory   = 'C:\Users\snb10\Desktop\GitHubRepos\IVSG\FieldDataCollection\DataCollectionProcedures\LoadRawDataToMATLAB\Functions\fcn_LoadRawDataToMATLAB_loadRawData';
sourceDirectory = 'C:\Users\snb10\Desktop\GitHubRepos\IVSG\FieldDataCollection\DataCollectionProcedures\LoadRawDataToMATLAB\Functions';
% Get a list of all files in the directory
fileList = dir(fullfile(sourceDirectory, '*.*')); % Adjust file extension as needed

% Filter out directories from the list
fileList = fileList(~[fileList.isdir]);

for ith_file = 1:length(fileList)
    sourceFileName = fileList(ith_file).name;
    revisedSourceFileName = replace(sourceFileName,newString,oldString);
    newFileNameString = revisedSourceFileName(1:end-2);

    % Call the function
    flagStringWasFound = fcn_DebugTools_findStringInDirectory(directoryPaths, newFileNameString, directoryQuery, figNum);
    
    if flagStringWasFound
        % Ask to replace
        mainMenuChoice = input('Replace string in this directory? [default = n]:','s');
        if isempty(mainMenuChoice)
            mainMenuChoice = 'n';
        end
        % Replace if yes
        if strcmpi(mainMenuChoice,'y')            
            stringToReplaceWith = replace(newFileNameString,oldString,newString);
            flagSkipComments = 2; % Keep the "% As:" comments            
            fcn_DebugTools_replaceStringInDirectory(directoryPaths, newFileNameString, stringToReplaceWith, flagSkipComments);
        end
            
    end

end