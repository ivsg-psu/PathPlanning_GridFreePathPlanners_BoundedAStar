function weight_matrix = fcn_make_potential_field_weight_matrix(vgraph, all_pts)
    weight_matrix = ones(size(vgraph,1), size(vgraph,2));
    syms F(x,y) r(t)
    F(x,y) = [-(x-0.4);
              -(y-0.4);
                0];
    for i = 1:size(vgraph,1)
        P1 = all_pts(i,1:3); % start point
        for j = 1:size(vgraph,2)
            P2 = all_pts(2,1:3); % end point
            r = P1 + t*(P2-P1); % parameterization of the line segment from P1 to P2
            Fr = F(r);
            dr = diff(r(t),t);
            integrand = dot(Fr,dr);
            weight = int(integrand,t,[0 1]);
            weight_matrix(i,j) = weight;
        end
    end
    %% numeric integral version of this algo with explicit t
    % t = (0:0.1:1)''; % line segment will be parameterized as r(t)
    % for i = 1:size(vgraph,1)
    %     P1 = all_pts(i,1:3); % start point
    %     for j = 1:size(vgraph,2)
    %         P2 = all_pts(2,1:3); % end point
    %         r = ones(length(t),1)*P1 + t*(P2-P1); % parameterization of the line segment from P1 to P2
    %     end
    % end
    %% matlab example
    % syms F1(x,y) F2(x,y) r1(t) r2(t)
    % r1(t) = cos(t)+2;
    % r2(t) = sin(t);
    % F1 = -y;
    % F2 = x;
    % r = [r1;r2];
    % F = [F1;F2];
    % Fr = F(r1(t),r2(t));
    % dr = diff(r(t),t);
    % integrand = dot(Fr,dr);
    % work = int(integrand,t,[0,2*pi])
end
