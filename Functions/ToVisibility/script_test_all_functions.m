%% script_test_all_functions.m
% This is a wrapper script to run all the test scripts in the 
% library for the purpose of evaluating every assertion test in these
% files.
%
% NOTE: to view the output file with formatting, use the "type" command.
% For example:
% type('script_test_fcn_Laps_all_stdout.txt')

clearvars; 
close all; 
clc;
all_scripts = dir(cat(2,'.',filesep,'Functions',filesep,'script_test_fcn_Visibility_*.m'));
N_files = length(all_scripts);
testing_times = nan(N_files,1);

diary 'script_test_fcn_Visibility_all_stdout.txt';

for i_script = 1:N_files
    file_name_extended = all_scripts(i_script).name;
    file_name = erase(file_name_extended,'.m');
    if ~strcmp(mfilename,file_name)
        %file_name_trunc = erase(file_name,'script_');
        fcn_DebugTools_cprintf('*blue',' ');
        fcn_DebugTools_cprintf('*blue','Testing script: %.0d of %.0d, %s\n\n',i_script,length(all_scripts),file_name);
        % disp('Press any key to continue');
        tstart = tic;
        suite = testsuite(file_name);
        results = run(suite);
        telapsed = toc(tstart);
        testing_times(i_script) = telapsed;
    end
end
diary off

close all;
figure(458908);
plot(testing_times);
grid on;
xlabel('Script test number');
ylabel('Elapsed time to test (sec)');

fprintf(1,'The testing times for each script:\n');
for i_script = 1:N_files
    if ~isnan(testing_times(i_script))
        fprintf(1,'%.0d: \t %.2f seconds for script: \t %s \n',i_script, testing_times(i_script), all_scripts(i_script).name);
    end
end