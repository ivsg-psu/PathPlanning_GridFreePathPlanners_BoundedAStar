% Ideas to speed up:
% - sort the list when new costs arrive (DID NOT WORK)
% - flush the list to remove infeasible from future tests, to avoid looping
%   (DOES NOT WORK - List is inherently sparse unless tests are done or
%   unless a lower cost is found)
% - loop through cities visited, and pierce list regularly to update global
% lowest costs (use greedy) 
% - use the queue list to directly do updates,
% not rows to add structure.
%   This won't make HUGE speed improvments, as it is a MATLAB memory/code
%   management issue, not algorithm optimization.
% - use prior sequencing to better predict future costs, e.g. use paths
%   completed so far to better estimate costs ahead (better empty cost
%   calculation)
% - group points. Namely - group clusters of points together into one.

%%%%%%%%%%%%%%%%%%%%%%
% RESULTS before adding list sorting:
% Saved as: fcn_BoundedAStar_solveTSP_v00001
% FINAL RESULTS:
% Number of cities (goals, no start): 8
% Wall time: 0.467 seconds
% Solution sequence: 	1 6 9 3 7 5 4 2 8 1 
% Number of main loop iterations: 33627
% Number of staged tests checked: 62106
% Number of staged tests: 62550
%
% FINAL RESULTS:
% Number of cities (goals, no start): 9
% Wall time: 2.433 seconds
% Solution sequence: 	1 6 9 3 7 5 4 10 2 8 1 
% Number of main loop iterations: 164301
% Number of staged tests checked: 345861
% Number of staged tests: 347077
% 
% FINAL RESULTS:
% Number of cities (goals, no start): 10
% Wall time: 8.641 seconds
% Solution sequence: 	1 6 9 3 7 11 5 4 10 2 8 1 
% Number of main loop iterations: 700,737
% Number of staged tests checked: 1,686,255
% Number of staged tests: 1,688,083
%
% Approximately a factor of 5 increase with each. 20 more cities.
% 5^20 = 100,000,000,000,000. This is the factor by which the speed needs
% to be improved.

%%%%%%%%%%%%
% Try sorting: fcn_BoundedAStar_solveTSP_v00002
% (starting from fcn_BoundedAStar_solveTSP_v00001)
% - sort the list when new costs arrive
% FINAL RESULTS:
% Number of cities (goals, no start): 10
% Wall time: 9.036 seconds
% Solution sequence: 	1 6 9 3 7 11 5 4 10 2 8 1 
% Number of main loop iterations: 698373
% Number of staged tests checked: 1,531,408
% Number of staged tests: 1,685,719
% CONCLUSION: slower, only a bit, and still main work is in checking staged
% tests that are useless. Need to sort solutions more regularly.

%%%%%%%%%%%%
% Try sorting periodically (every N): fcn_BoundedAStar_solveTSP_v00003
% (starting from fcn_BoundedAStar_solveTSP_v00002)
% - sort the list every 5000
% FINAL RESULTS:
% Number of cities (goals, no start): 10
% Wall time: 30.943 seconds
% Solution sequence: 	1 6 9 3 7 11 5 4 10 2 8 1 
% Number of main loop iterations: 492332
% Number of staged tests checked: 492647
% Number of staged tests: 1253251
% CONCLUSION: MUCH slower. Spent much more time sorting, but never hit good
% solutions. This suggests that sorting is not a great primary strategy.
% Need a way to prune costs quickly to find good solutions fast.





for ith_iteration = 1:10
    % Call TSP function
    [orderedVisitSequence] = fcn_BoundedAStar_solveTSP(...
        startAndGoalPoints, costsFromTo, pathsFromTo, (cellArrayOfFunctionOptions), (-1));
end