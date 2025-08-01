rawdata = [1; 2; 3; 4; 2; 7];
y_init = [1; 2];
B = [0.067455273889072   0.134910547778144   0.067455273889072];
A = [1.000000000000000  -1.142980502539901   0.412801598096189];
Aflipped = fliplr(A);


% Implements:
% a(1)*y(n) = b(1)*x(n) + b(2)*x(n-1) + ... + b(nb+1)*x(n-nb)
%                           - a(2)*y(n-1) - ... - a(na+1)*y(n-na)
% with explicit initial conditions on y

Ndata = length(rawdata(:,1));

% Amatrix = zeros(Ndata,Ndata);
% for ith_a = 2:length(A)
%     avector = A(ith_a)*ones(Ndata-(ith_a-1),1);
%     diagonalAddition = diag(avector,(1-ith_a));
%     Amatrix = Amatrix+diagonalAddition;
% end


Bmatrix = zeros(Ndata,Ndata);
for ith_b = 1:length(B)
    bvector = B(ith_b)*ones(Ndata-(ith_b-1),1);
    diagonalAddition = diag(bvector,(1-ith_b));
    Bmatrix = Bmatrix+diagonalAddition;
end


yfilt_hardcoded = Bmatrix*rawdata;

% Fill in initial conditions:
yfilt_hardcoded(1:2,:) = y_init;
% yfilt_hardcoded(2,1) = yfilt_hardcoded(2,1) - A(1,2)*yfilt_hardcoded(1,1);

order = length(A)-1;
for ith_sample = (order+1):Ndata
    yfilt_hardcoded(ith_sample,1) = yfilt_hardcoded(ith_sample,1) - Aflipped(1:order)*yfilt_hardcoded((ith_sample-order):(ith_sample-1),:);
end

yfilt = filter(B,A,rawdata);

