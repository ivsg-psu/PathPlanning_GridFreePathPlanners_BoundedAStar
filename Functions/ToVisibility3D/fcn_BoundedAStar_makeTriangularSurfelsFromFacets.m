function all_surfels = fcn_BoundedAStar_makeTriangularSurfelsFromFacets(time_space_polytopes, varargin)
% fcn_BoundedAStar_makeTriangularSurfelsFromFacets
%
% Decomposes timespace polytopes with facets into triangular surface elements (surfels) which are necessary for
% performing intersection checking between possible visibity graph edges and polytope obstacles.
%
% FORMAT:
% all_surfels = fcn_BoundedAStar_makeTriangularSurfelsFromFacets(time_space_polytopes, fig_num)
%
% INPUTS:
%
%     time_space_polytopes: this is the time space polytope struct array with
%     both vertices and facets, of the form output by fcn_BoundedAStar_makeFacetsFromVerts
%     i.e. a struct array with the following fields:
%       vertices: field holding the vertices of each polytope
%           vertices consists of 4 columns: x position, y position, time (z-axis position) and vertex id
%           the vertex ID is necessary for correctly mapping a vertex at one time to its position at the next time
%       flats: field associating vertices into flat facets (i.e. facets that lie flat in a single time plane)
%       sides: field associating vertices into side wall facets (i.e. facets that can span several time values)
%       both the flats and sides fields have the same format: a matrix where each row is a facet and every 4 columns
%         is one vertex consisting of (x, y, t, and ID) as stated above
%         e.g. two flats with three vertices each would be represented as the following 2x12 matrix:
%         flat 1, vert 1 x, flat 1 vert 1 y, flat 1 vert 1 t, flat 1 vert 1 ID, ..., flat 1, vert 3 x, flat 1 vert 3 y, flat 1 vert 3 t, flat 1 vert 3 ID
%         flat 2, vert 1 x, flat 2 vert 1 y, flat 2 vert 1 t, flat 2 vert 1 ID, ..., flat 2, vert 3 x, flat 2 vert 3 y, flat 2 vert 3 t, flat 2 vert 3 ID
%
%     (optional inputs)
%
%     fig_num: a figure number to plot results. If set to -1, skips any
%     input checking or debugging, no figures will be generated, and sets
%     up code to maximize speed. As well, if given, this forces the
%     variable types to be displayed as output and as well makes the input
%     check process verbose
%
%
% OUTPUTS:
%     all_surfels: a matrix with all triangular surface elements (surfels) from all timespace polytopes
%     there is one row for each surfel
%     each row has 9 columns representing the x,y,t coordinates of each point of the triangle ordered
%     x1 y1 t1 x2 y2 t2 x3 y3 t3
%
% DEPENDENCIES:
% generally, fcn_BoundedAStar_makeFacetsFromVerts can be run before this function to create facets for this function to triangulate
% see scritp_test_3d_polytope_multiple or the readme for an example of the typical call order
% but there are no dependencies in the source code of this function
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
% 2025_07_17 - K. hayes, kxh1031@psu.edu
% -- copied to new function from fcn_make_triangular_surfels_from_facets to
%    follow library convention
% 2025_08_15 - K. Hayes
% -- updated fcn header and formatting
% -- moved plotting into fcn debug section
%
% TO DO:
%
% -- fill in to-do items here.

%% Debugging and Input checks
% Check if flag_max_speed set. This occurs if the fig_num variable input
% argument (varargin) is given a number of -1, which is not a valid figure
% number.
MAX_NARGIN = 2; % The largest Number of argument inputs to the function
flag_max_speed = 0;
if (nargin==MAX_NARGIN && isequal(varargin{end},-1))
    flag_do_debug = 0; %     % Flag to plot the results for debugging
    flag_check_inputs = 0; % Flag to perform input checking
    flag_max_speed = 1;
else
    % Check to see if we are externally setting debug mode to be "on"
    flag_do_debug = 0; %     % Flag to plot the results for debugging
    flag_check_inputs = 1; % Flag to perform input checking
    MATLABFLAG_MAPGEN_FLAG_CHECK_INPUTS = getenv("MATLABFLAG_MAPGEN_FLAG_CHECK_INPUTS");
    MATLABFLAG_MAPGEN_FLAG_DO_DEBUG = getenv("MATLABFLAG_MAPGEN_FLAG_DO_DEBUG");
    if ~isempty(MATLABFLAG_MAPGEN_FLAG_CHECK_INPUTS) && ~isempty(MATLABFLAG_MAPGEN_FLAG_DO_DEBUG)
        flag_do_debug = str2double(MATLABFLAG_MAPGEN_FLAG_DO_DEBUG);
        flag_check_inputs  = str2double(MATLABFLAG_MAPGEN_FLAG_CHECK_INPUTS);
    end
end

% flag_do_debug = 1;

if flag_do_debug
    st = dbstack; %#ok<*UNRCH>
    fprintf(1,'STARTING function: %s, in file: %s\n',st(1).name,st(1).file);
    debug_fig_num = 999978; %#ok<NASGU>
else
    debug_fig_num = []; %#ok<NASGU>
end

%% check input arguments?
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   _____                   _
%  |_   _|                 | |
%    | |  _ __  _ __  _   _| |_ ___
%    | | | '_ \| '_ \| | | | __/ __|
%   _| |_| | | | |_) | |_| | |_\__ \
%  |_____|_| |_| .__/ \__,_|\__|___/
%              | |
%              |_|
% See: http://patorjk.com/software/taag/#p=display&f=Big&t=Inputs
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if 0==flag_max_speed
    if flag_check_inputs
        % Are there the right number of inputs?
        narginchk(1,MAX_NARGIN);

        % Check the time_space_polytopes input, make sure it is struct
        assert(isstruct(time_space_polytopes));
    end
end

% Does user want to specify all_pts?
% all_pts = []; % Default is to NOT show plots
% if 2 <= nargin
%     temp = varargin{1};
%     if ~isempty(temp) % Did the user NOT give an empty figure number?
%        all_pts = temp;
%     end
% end

% Does user want to show the plots?
flag_do_plots = 0; % Default is to NOT show plots
if (0==flag_max_speed) && (MAX_NARGIN == nargin) 
    temp = varargin{end};
    if ~isempty(temp) % Did the user NOT give an empty figure number?
        fig_num = temp;
        figure(fig_num);
        flag_do_plots = 1;
    end
end


%% Main code
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   __  __       _
%  |  \/  |     (_)
%  | \  / | __ _ _ _ __
%  | |\/| |/ _` | | '_ \
%  | |  | | (_| | | | | |
%  |_|  |_|\__,_|_|_| |_|
%
%See: http://patorjk.com/software/taag/#p=display&f=Big&t=Main
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%§


    all_surfels = [];
    % loop through each polytope
    for i = 1:length(time_space_polytopes)
        flats = time_space_polytopes(i).flats; % extract flats and sides from polytope
        sides = time_space_polytopes(i).sides;
        % loop through each flat, representing a 2D polytope at one time (see fcn_BoundedAStar_makeFacetsFromVerts)
        for j = 1:size(flats,1)
            my_flat = flats(j,:); % store flat of interest
            my_flat_matrix = (reshape(my_flat,[4 length(my_flat)/4]))'; % unwrap flat from a vector into a matrix with columns for (x y t id)
            flat_centroid = [mean(my_flat_matrix(:,1)) mean(my_flat_matrix(:,2)) mean(my_flat_matrix(:,3))]; % find centroid of the flat
            my_flat_polyshape = polyshape(my_flat_matrix(:,1),my_flat_matrix(:,2)); % convert flat polytope to a polyshape
            my_flat_triangulated = triangulation(my_flat_polyshape); % perform Delaunay triangulation on the polyshape
            tris_this_flat = [];
            % each iteration of the following loop takes the outputs of MATLAB's Delaunay triangulation
            % function and reformats them as a matrix of vertices representing the triangles
            for k=1:size(my_flat_triangulated.ConnectivityList,1)
                x1 = my_flat_triangulated.Points(my_flat_triangulated.ConnectivityList(k,1),1);
                y1 = my_flat_triangulated.Points(my_flat_triangulated.ConnectivityList(k,1),2);
                x2 = my_flat_triangulated.Points(my_flat_triangulated.ConnectivityList(k,2),1);
                y2 = my_flat_triangulated.Points(my_flat_triangulated.ConnectivityList(k,2),2);
                x3 = my_flat_triangulated.Points(my_flat_triangulated.ConnectivityList(k,3),1);
                y3 = my_flat_triangulated.Points(my_flat_triangulated.ConnectivityList(k,3),2);
                tris_this_flat = [tris_this_flat; x1 y1 my_flat_matrix(1,3) x2 y2 my_flat_matrix(2,3) x3 y3 my_flat_matrix(3,3)];
            end
            all_surfels = [all_surfels; tris_this_flat]; % add triangles from this flat to all surfel array
            % fill3(my_flat_matrix(:,1),my_flat_matrix(:,2),my_flat_matrix(:,3),rand(1,3),'FaceAlpha',0.3);
        end
        % loop through each side wall of the 3D timespace obstacle
        % triangulating this is simple as each side wall is a parallelogram (see fcn_BoundedAStar_makeFacetsFromVerts)
        % thus vertices can just be grouped, there is no need to use Delaunay
        for j = 1:size(sides,1)
            my_side = sides(j,:); % store side of interest
            my_side_matrix = (reshape(my_side,[4 length(my_side)/4]))'; % unwrap side from a vector into a matrix with columns for (x y t id)
            tris_this_side = [my_side_matrix(1,1:3) my_side_matrix(2,1:3) my_side_matrix(3,1:3);...
                              my_side_matrix(1,1:3) my_side_matrix(3,1:3) my_side_matrix(4,1:3)]; % 1 2 3, 1 3 4
            all_surfels = [all_surfels; tris_this_side];
            % fill3(my_side_matrix(:,1),my_side_matrix(:,2),my_side_matrix(:,3),rand(1,3),'FaceAlpha',0.3);
        end
    end
%% Plot the results (for debugging)?
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   _____       _
%  |  __ \     | |
%  | |  | | ___| |__  _   _  __ _
%  | |  | |/ _ \ '_ \| | | |/ _` |
%  | |__| |  __/ |_) | |_| | (_| |
%  |_____/ \___|_.__/ \__,_|\__, |
%                            __/ |
%                           |___/
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 

if flag_do_plots
    % set up plot
    figure(fig_num);
    hold on;
    view(3)
    box on;
    INTERNAL_fcn_format_timespace_plot();
    
     % Plot polytopes in timespace
    for i = 1:size(all_surfels,1)
        fill3([all_surfels(i,1) all_surfels(i,4) all_surfels(i,7)], [all_surfels(i,2) all_surfels(i,5) all_surfels(i,8)], [all_surfels(i,3) all_surfels(i,6) all_surfels(i,9)],rand(1,3),'FaceAlpha',0.3,'HandleVisibility','off');
    end

end 

end % end function

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
function INTERNAL_fcn_format_timespace_plot()
    % define figure properties
    % opts.width      = 8.8;
    % opts.height     = 6;
    % opts.fontType   = 'Times New Roman';
    % opts.fontSize   = 14;
    % fig = gcf;
    % % scaling
    % fig.Units               = 'centimeters';
    % fig.Position(3)         = opts.width;
    % fig.Position(4)         = opts.height;
    
    % % set text properties
    % set(fig.Children, ...
    %     'FontName',     'Times New Roman', ...
    %     'FontSize',     14);
    
    % remove unnecessary white space
    set(gca,'LooseInset',max(get(gca,'TightInset'), 0.02))
    xlabel('x [km]')
    ylabel('y [km]')
    zlabel('t [min]')
    view([36 30])
end