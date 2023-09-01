% script_test_fcn_make_potential_field_weight_matrix
num_pts = 20;
vgraph = ones(num_pts,num_pts);
x = rand(num_pts,1);
y = rand(num_pts,1);
z = rand(num_pts,1);
all_pts = [x,y,z];
figure; box on;
title('points for testing')
plot3(x,y,z,'xk')

tic
weight_matrix_numeric = fcn_make_potential_field_weight_matrix(vgraph, all_pts, 'numeric');
toc
total_weight_numeric = sum(sum(weight_matrix_numeric))
avg_weight_per_el_numeric = total_weight_numeric/(num_pts*num_pts)

tic
weight_matrix_symbolic = fcn_make_potential_field_weight_matrix(vgraph, all_pts, 'symbolic');
toc
total_weight_symbolic = sum(sum(weight_matrix_symbolic))
avg_weight_per_el_symbolic = total_weight_symbolic/(num_pts*num_pts)

pct_error_of_numeric_from_total = (total_weight_numeric-total_weight_symbolic)/total_weight_symbolic*100
pct_error_of_numeric_from_avg_el = (avg_weight_per_el_numeric-avg_weight_per_el_symbolic)/avg_weight_per_el_symbolic*100
