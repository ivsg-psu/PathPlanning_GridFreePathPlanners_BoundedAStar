%% script_test_all_functions.m
% This is a wrapper script to run all the test scripts in the 
% library for the purpose of evaluating every assertion test in these
% files.
%
% NOTE: to view the output file with formatting, use the "type" command.
% For example:
% type('script_test_fcn_geometry_all_stdout.txt')

% REVISION HISTORY:
% 2025_11_06 by Sean Brennan
% -- Started revision history 
% -- Updated clc and clear all checking to avoid checking this file
% -- Added subfunction (INTERNAL) to remove specific file names from
%    % checking
% -- Improved error checking for missed functions and test scripts
% 2025_11_07 by Sean Brennan
% -- Added root folder checks
%    % * that the demo file scrit_demo_(reponame).m exists
%    % * that number of mfiles is only one in the root directory
%    % * that the Functions subfolder exists

% clearvars; 
close all; 
clc;

repoShortName = '_VGraph_';
% repoShortName = '_DebugTools_';


%% Check the root folder 
% that the expected demo file exists
% that number of mfiles is only one in the root directory

rootDirectoryQuery = fullfile(pwd,'*.m');
rootFolderFileList = dir(rootDirectoryQuery);

expectedNameString = cat(2,'script_demo',repoShortName(1:end-1),'.m');
if ~exist(expectedNameString,'file')
    fcn_DebugTools_cprintf('*red','The following demo file was expected in the root folder, but was not found in the root directory: \n\t%s\n', expectedNameString);
end    

if 1~=length(rootFolderFileList)
    fcn_DebugTools_cprintf('*red','The following files were found in the root folder, but only one demo script was expected: %s\n', expectedNameString);
    for ith_file = 1:length(rootFolderFileList)
        currentFileIndex = ith_file;
        fcn_DebugTools_cprintf('*red','\t%s\n',rootFolderFileList(currentFileIndex).name)
    end    
end

if ~exist('Functions','dir')
    fcn_DebugTools_cprintf('*red','The Functions subfolder was expected below the root folder, but was not found?');
end    

%% Check the Functions folder (hereafter)

functionDirectoryQuery = fullfile(pwd,'Functions','*.*');
% Use the following instead, if wish to do subdirectories
% directoryQuery = fullfile(pwd,'Functions','**','*.*');

functionFolderFileList = dir(functionDirectoryQuery); %cat(2,'.',filesep,filesep,'script_test_fcn_*.m'));


% Filter out directories from the list
functionFolderFileList = functionFolderFileList(~[functionFolderFileList.isdir]);

%% Make sure there's no 'clc' or 'clear all' commands
queryString = 'clc';
flagsStringWasFoundInFilesRaw = fcn_DebugTools_directoryStringQuery(functionFolderFileList, queryString, (-1));
flagsStringWasFoundInFiles = fcn_INTERNAL_removeFromList(flagsStringWasFoundInFilesRaw, functionFolderFileList,'script_test_all_functions');
if sum(flagsStringWasFoundInFiles)>0
    fcn_DebugTools_directoryStringQuery(functionFolderFileList, queryString, 1);
    error('A "clc" command was found in one of the functions other than script_test_all_functions - see listing above. This needs to be fixed before continuing because clc commands remove warnings and errors shown during function testing.');
end

queryString = 'clear all';
flagsStringWasFoundInFilesRaw = fcn_DebugTools_directoryStringQuery(functionFolderFileList, queryString, (-1));
flagsStringWasFoundInFiles = fcn_INTERNAL_removeFromList(flagsStringWasFoundInFilesRaw, functionFolderFileList,'script_test_all_functions');
if sum(flagsStringWasFoundInFiles)>1
    fcn_DebugTools_directoryStringQuery(functionFolderFileList, queryString, 1);
    error('A "clear all" command was found in one of the functions other than script_test_all_functions - see listing above. This needs to be fixed before continuing because clc commands remove warnings and errors shown during function testing.');
end

%% Match functions to scripts

N_files = length(functionFolderFileList);
testing_times = nan(N_files,1);

outputFile = cat(2,'script_test_fcn',repoShortName,'all_stdout.txt');
diary(fullfile(pwd,outputFile));

flags_isFile  = zeros(N_files,1); % All files (excludes directories)

flags_isMfile = zeros(N_files,1); % File has a .m extension
flags_isMfileFunction = zeros(N_files,1); % File starts with fcn_
flags_isMfileRepeated = zeros(N_files,1); % File is repeated between folders
flags_isMfileTestedFunction = zeros(N_files,1); % File is a fcn_XXX and there's a script_test_XXX that tests it

flags_isMfileTestingScript = zeros(N_files,1); % File starts with script_test_fcn_
flags_isMfileTestingScriptWithMatchingFunction = zeros(N_files,1); % File is a script_test_XXX that matches a function

% Check all the files to see which ones should be tested
for i_script = 1:N_files
    % Is this a file?
    if ~functionFolderFileList(i_script).isdir
        flags_isFile(i_script,1) = 1;
    end
    
    file_name_extended = functionFolderFileList(i_script).name;
    file_directory = functionFolderFileList(i_script).folder;
    
    % Is this an m-file?
    if (1==flags_isFile(i_script,1) ) && length(file_name_extended)>2 && strcmp(file_name_extended(end-1:end),'.m')
        flags_isMfile(i_script,1) = 1;

        % Is this a repeated m-file?
        for jth_file = 1:N_files
            nameToTest = functionFolderFileList(jth_file).name;
            if (i_script~=jth_file) && strcmp(file_name_extended,nameToTest)
                flags_isMfileRepeated(jth_file,1) = 1;
            end
        end
    end

    % Is this an m-file function?
    if (1==flags_isMfile(i_script,1)) && length(file_name_extended)>7 && strcmp(file_name_extended(1:4),'fcn_')
        flags_isMfileFunction(i_script,1) = 1;

        % Does this m-file function have a matching test script?
        fullPath = which(file_name_extended);
        testName = cat(2,'script_test_',file_name_extended);
        testFullPathName = fullfile(file_directory,testName);
        for jth_file = 1:N_files
            listedFullName = fullfile(functionFolderFileList(jth_file).folder,functionFolderFileList(jth_file).name);
            if strcmp(testFullPathName,listedFullName)
                flags_isMfileTestedFunction(i_script,1) = 1;
            end
        end

    end   

    % Is this an m-file testing script?
    if (1==flags_isMfile(i_script,1) ) && length(file_name_extended)>19 && strcmp(file_name_extended(1:16),'script_test_fcn_')
        flags_isMfileTestingScript(i_script,1) = 1;

        % Does this testing script match to a function?
        testMfileName = file_name_extended(13:end);
        testFullPathName = fullfile(file_directory,testMfileName);
        for jth_file = 1:N_files
            listedFullName = fullfile(functionFolderFileList(jth_file).folder,functionFolderFileList(jth_file).name);
            if strcmp(testFullPathName,listedFullName)
                flags_isMfileTestingScriptWithMatchingFunction(i_script,1) = 1;
            end
        end
    end

end

flags_isEitherTestScriptOrTestedFunction = flags_isMfileTestedFunction+flags_isMfileTestingScriptWithMatchingFunction;

%% Summarize results
fprintf(1,'\nSUMMARY OF FOUND FILES: \n');
indicies_filesToTest = find(1==flags_isMfileTestingScriptWithMatchingFunction);
if ~isempty(indicies_filesToTest)
    fcn_DebugTools_cprintf('*blue','The following scripts were found that will be tested:\n');
    for ith_file = 1:length(indicies_filesToTest)
        currentFileIndex = indicies_filesToTest(ith_file);
        fprintf(1,'\t%s\n',functionFolderFileList(currentFileIndex).name)
    end    
end

% List missed files
indicies_missedFiles_flags_RAW = flags_isFile.*(0==flags_isMfile);
indicies_missedFiles_flags = fcn_INTERNAL_removeFromList(indicies_missedFiles_flags_RAW, functionFolderFileList,'script_test_all_functions');
indicies_missedFiles = find(indicies_missedFiles_flags);
if ~isempty(indicies_missedFiles)
    fcn_DebugTools_cprintf('*red','The following files were found, but do not seem to be repo functions or scripts:\n');
    for ith_file = 1:length(indicies_missedFiles)
        currentFileIndex = indicies_missedFiles(ith_file);
        fcn_DebugTools_cprintf('*red','\t%s\n',functionFolderFileList(currentFileIndex).name)
    end    
end

% List mfiles that are not testing scripts or functions
indicies_missedMfiles_flags_RAW = flags_isMfile.*(0==flags_isMfileFunction).*(0==flags_isMfileTestingScript);
indicies_missedMfiles_flags = fcn_INTERNAL_removeFromList(indicies_missedMfiles_flags_RAW, functionFolderFileList,'script_test_all_functions');
indicies_missedMfiles = find(indicies_missedMfiles_flags);

if ~isempty(indicies_missedMfiles)
    fcn_DebugTools_cprintf('*red','The following m-files were found, but do not seem to be test scripts or functions:\n');
    for ith_file = 1:length(indicies_missedMfiles)
        currentFileIndex = indicies_missedMfiles(ith_file);
        fcn_DebugTools_cprintf('*red','\t%s\n',functionFolderFileList(currentFileIndex).name)
    end    
end

% List missed functions
indicies_missedFunctions = find(1==(flags_isMfileFunction.*(0==flags_isMfileTestedFunction)));
if ~isempty(indicies_missedFunctions)
    fcn_DebugTools_cprintf('*red','The following functions were found, but do not have a matching test scripts:\n');
    for ith_file = 1:length(indicies_missedFunctions)
        currentFileIndex = indicies_missedFunctions(ith_file);
        fcn_DebugTools_cprintf('*red','\t%s\n',functionFolderFileList(currentFileIndex).name)
    end    
end

% List missed scripts
indicies_missedScripts = find(1==(flags_isMfileTestingScript.*(0==flags_isMfileTestingScriptWithMatchingFunction)));
if ~isempty(indicies_missedScripts)
    fcn_DebugTools_cprintf('*red','The following test scripts were found, but do not have a matching function:\n');
    for ith_file = 1:length(indicies_missedScripts)
        currentFileIndex = indicies_missedScripts(ith_file);
        fcn_DebugTools_cprintf('*red','\t%s\n',functionFolderFileList(currentFileIndex).name)
    end    
end

% List repeats
indicies_repeatedFiles = find(1==flags_isMfileRepeated);
if ~isempty(indicies_repeatedFiles)
    fcn_DebugTools_cprintf('*red','The following files seem to be repeated:\n');
    for ith_file = 1:length(indicies_repeatedFiles)
        currentFileIndex = indicies_repeatedFiles(ith_file);
        fcn_DebugTools_cprintf('*red','\t%s\n',functionFolderFileList(currentFileIndex).name)
    end    
end

%% Make sure all functions have correct global variables
temp = functionFolderFileList(indicies_filesToTest);
for ith_file = 1:length(temp)
    temp(ith_file).name = extractAfter(temp(ith_file).name,'script_test_');
end
queryString = upper(repoShortName);
flagsStringWasFoundInFiles = fcn_DebugTools_directoryStringQuery(temp, queryString, (-1));
if ~all(flagsStringWasFoundInFiles)
    fcn_DebugTools_directoryStringQuery(temp, queryString, 1);
    warning('backtrace','on');
    warning('The global variable: %s is expected in every file, but was not found. See above listing.',queryString);
end


%% Test the good scripts
NtestScripts = length(indicies_filesToTest);
fprintf(1,'\nSTARTING TESTS:\n');
allResults = cell(NtestScripts,1);
for ith_testScript = 1:NtestScripts
    currentFileIndex = indicies_filesToTest(ith_testScript);
    file_name_extended = functionFolderFileList(currentFileIndex).name;
    file_name = erase(file_name_extended,'.m');
    file_name_trunc = erase(file_name,'script_');
    fcn_DebugTools_cprintf('*blue','%s\n','   ');
    fcn_DebugTools_cprintf('*blue','Testing script: %.0d of %.0d, %s\n\n',ith_testScript,NtestScripts,file_name_trunc);

    % Start the test
    tstart = tic;
    suite = testsuite(file_name);
    allResults{ith_testScript,1} = run(suite);
    telapsed = toc(tstart);
    testing_times(ith_testScript) = telapsed;
end
diary off

close all;
figure(1)
plot(testing_times);
grid on;
xlabel('Script test number');
ylabel('Elapsed time to test (sec)');




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

%% fcn_INTERNAL_removeFromList
function newFlags = fcn_INTERNAL_removeFromList(flagsToFix, fileList,fileToRemove)
% Remove this function from list
newFlags = flagsToFix;
for ith_file = 1:length(fileList)
    if contains(fileList(ith_file).name,fileToRemove)
        newFlags(ith_file) = false;
    end
end
end % Ends fcn_INTERNAL_removeFromList