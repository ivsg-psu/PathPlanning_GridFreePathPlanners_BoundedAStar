function new_cost_polytopes = ...
    fcn_polytope_editing_set_all_costs(polytopes,des_cost)
    for poly_idx = 1:length(polytopes)
        polytopes(poly_idx) = setfield(polytopes(poly_idx),'cost',des_cost);
    end
    new_cost_polytopes = polytopes;
end
