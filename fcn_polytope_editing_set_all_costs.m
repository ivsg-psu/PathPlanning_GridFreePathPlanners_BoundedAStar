function new_cost_polytopes = ...
    fcn_polytope_editing_set_all_costs(polytopes,des_cost)
    % fcn_polytope_editing_set_all_costs
    % sets the cost of all obstacles in a field to a desired cost value
    %
    % FORMAT:
    %
    % new_cost_polytopes = ...
    % fcn_polytope_editing_set_all_costs(polytopes,des_cost)
    %
    % INPUTS:
    %
    % POLYTOPES: a 1-by-n seven field structure of shrunken polytopes,
    % where n <= number of polytopes with fields:
    %   vertices: a m+1-by-2 matrix of xy points with row1 = rowm+1, where m is
    %     the number of the individual polytope vertices
    %   xv: a 1-by-m vector of vertice x-coordinates
    %   yv: a 1-by-m vector of vertice y-coordinates
    %   distances: a 1-by-m vector of perimeter distances from one point to the
    %     next point, distances(i) = distance from vertices(i) to vertices(i+1)
    %   mean: centroid xy coordinate of the polytope
    %   area: area of the polytope
    % des_cost: value of cost of polytope traversal in fraction above free space traversal cost
    % i.e. a des_cost of 0.2 means the traversal cost of crossing a polytope is 1.2 times harder
    % than traversing free space outside of polytopes
    %
    % OUTPUTS:
    %
    %
    % NEW_COST_POLYTOPES: a new polytope structure containing the same polytopes, with their costs
    %   modified accordingly
    % DEPENDENCIES:
    %
    % EXAMPLES:
    %
    % For additional examples, see: script_planning_performed_at_multiple_costs.m
    %
    % This function was written in 2022_05 by Steve Harentt
    % Questions or comments? sjh6473@psu.edu
    %

    % Revision History:

    % TO DO
    % -- it may be possibel to speed this up with setfield() or extract field instead of a loop
    % -- a good feature to add would be to allow for specification of a mean and standard deviation
    %    of costs so that a cost distribution can be applied instead of a fixed uniform cost

    for poly_idx = 1:length(polytopes)
        polytopes(poly_idx) = setfield(polytopes(poly_idx),'cost',des_cost);
    end
    new_cost_polytopes = polytopes;
end
