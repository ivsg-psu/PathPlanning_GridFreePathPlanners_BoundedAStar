function [is_reachable, num_steps, rgraph_total] = fcn_check_reachability(vgraph,start,finish)
 % fcn_check_reachability 
 % 
 % From the visibility graph describing node visible from each node,
 % finds and analyzes the reachability graph describing nodes that have a valid
 % multistep route from each node
 % 
 % 
 % 
 % FORMAT: 
 % [is_reachable, num_steps, rgraph_total] = fcn_check_reachability(vgraph,start,finish)
 %  
 % 
 % INPUTS: 
 % 
 %   start: the start point vector (x,t,t,id)
 % 
 %   finish: the finish point matrix of all valid finishes where each row is a single finish point vector (x,y,t,id)
 % 
 %   vgraph: the visibility graph as an nxn matrix where n is the number of points (nodes) in the map.
 %       A 1 is in position i,j if point j is visible from point i.  0 otherwise.
 % 
 % 
 % OUTPUTS: 
 % 
 %     is_reachable: binary set to 1 if the finish is reachable from the start in any number of steps.  0 otherwise.
 %
 %     num_steps: the minimum number of steps (path segments) required to reach finish from star
 % 
 %     rgraph_total: the total reachability graph as an nxn matrix where n is the number of pointes (nodes) in the map.
 %         A 1 is in position i,j if j is reachable from point i in a path with n or fewer steps (path segments). 0 otherwise.
 % 
 % DEPENDENCIES: 
 % 
 % none but several functions exist to create visibility matrices
 % 
 % EXAMPLES: 
 % 
 % See the script: script_test_3d_polytope_multiple 
 % for a full test suite. 
 % 
 % This function was written on summer 2023 by Steve Harnett 
 % Questions or comments? contact sjharnett@psu.edu 
  
 % 
 % REVISION HISTORY: 
 % 
 % 2023, summer by Steve Harnett 
 % -- first write of function 
 % 
 % TO DO: 
 % 
 % -- fill in to-do items here.
    num_pts = size(vgraph,1);
    start_id = start(4);
    finish_id = finish(:,4);
    start_id_repeated = ones(size(finish_id,1),size(finish_id,2))*start_id; % duplicate start IDs if multiple finishes
    is_reachable = 0; % initialize assuming not reachable
    rgraph_total = zeros(num_pts);
    % only want to check for paths with up to n steps because that is a path that uses every point once
    for num_steps = 1:num_pts
        % vgraph^k gives the rgraph describing reachability using paths with exactly k steps
        rgraph = vgraph^num_steps; % see Judith Gersting's Mathematical Structures for Computer Science for formula
        rgraph_total = rgraph_total + rgraph; % add rgraph for num_steps steps to all prior rgraphs with 1 to num_steps steps
        ind = sub2ind([num_pts,num_pts],start_id_repeated,finish_id); want to check all start finish combos
        if sum(rgraph(ind)) > 0
            is_reachable = 1; if any start to finish combo has more than 0, some finish is reachable
        end
    end
    rgraph_total = (rgraph_total>0); % make rgraph binary
end
