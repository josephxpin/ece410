%% Output 1:
% Print the matrices containing the bases of the given subspaces
disp('Output 1')

% defn of V, W
V = [1 -1 0 1 ; 1 1 0 0 ; 3 1 0 1]';
W = [1 0 2 1 ; 1 0 -2 0]';

% per lab handout, using orth function
V_basis = orth(V)
W_basis = orth(W)

%% Output 2:
% Print the matrices spanning the sum and intersection of the earlier
% vector spaces
disp('Output 2')

% dimensions must agree
% V's last column is linearly dependent on the first two columns
% So I will drop it

V = [1 -1 0 1 ; 1 1 0 0]';
W = [1 0 2 1 ; 1 0 -2 0]';

% function 1 - basis for the sum of two matrices
sum_basis = sub_sum(V, W)

% function 2 - basis for the intersection of two matrices
intersection_basis = sub_intersect(V, W)

%% Output 3:
% Print the coordinates z of the vector given in the given basis
% Verify numerically they're correct
disp('Output 3')

% defn o f x vectors
x1 = [1;1];
x2 = [2;1];

% construction of matrix A and defn of x
A = [x1 x2];
x = [2;1];

% soln for z
z = A\x

% checking solution
x_sol = z(1)*x1 + z(2)*x2;

if (x_sol == [2;1])
    disp('z is the correct coordinate representation of x = [2 1]T')
else
    disp('z is not the correct coordinate representation of x = [2 1]T')
end

%% Output 4:
% Find the expression of A_hat in terms of A, P, Q
% Find the matrix representation of the given transformation in the given
% bases
disp('Output 4')

% necessary to refresh variables
clear A;

% NB I use transpose to better illustrate the columns
% I prefer to see the columns laid out since much of this content regards
% the column space

A = transpose([1 0 1 0 0 ; 2 1 0 -1 0 ; 0 -1 3 2 2 ; -1 -2 4 3 2]);
P = transpose([1 0 0 0 ; 1 1 0 0 ; 1 0 1 0 ; 1 0 0 1]);
Q = transpose([1 1 0 0 0 ; 1 -1 0 0 0 ; 0 0 1 1 0 ; 0 0 1 -1 0 ; 1 0 0 0 1]);

% resultant equation from the math
% I assumed it was not expected to do the full procedure in the script
A_hat = inv(Q)*A*P

%% Output 5:
% multiple outputs required - see lab handout
disp('Output 5')

A = transpose([1 0 1 0 0 ; 2 1 0 -1 0 ; 0 -1 3 2 2 ; -1 -2 4 3 2]);

% I didn't want to hard code the m and n values, so I extract them
m = size(A,1);
n = size(A,2);

% rank and kernel computations
A_rank = rank(A)
nullity_A = n-A_rank

% flow statements verify the injective and surjective properties of A
if isequal(m,n,A_rank)
    disp('A is both injective and surjective')
elseif isequal(n,A_rank)
    disp('A is injective but not surjective')
elseif isequal(m,A_rank)
    disp('A is surjective but not injective')
else
    disp('A is neither injective nor surjective')
end

%% Output 6:
% determine the presence and uniqueness of a solution to
% Ax = b given in the lab handout
disp('Output 6')

clear x1 x2

% defn of b
b = [1;0;0;0;0]

% compute x1
x1 = solve(A,b);

% New b
b = [1;1;-2;-2;-2]

% compute x2
x2 = solve(A,b);

%% Output 7:
% Show A-invariance of the given subspace
% Print the matrix P of the representation matrix 
% Verify that inv(P)*A*P is upper triangular
disp('Output 7')

clear A

% re defn of A and V
A = [1 2 2 ; 1 -3 -5 ; -1 2 4];
V = transpose([0 1 -1 ; 1 -2 1]);

% determines if V is A-invariant
if isequal(rank([A*V V]),rank(V))
    disp('The subspace V is A-invariant')
else
    disp('The subspace V is NOT A-invariant')
end

% contructing P matrix
V_kernel = null(transpose(V));
W = orth(V_kernel);

P = [V W]

% confirming if P is upper triangular
checking_upper_triangular = inv(P)*A*P

%% Output 8:
% Print the Kalman decomposition for controllability for the given system
disp('Output 8')

clear A V V_kernel W

% construction of A and B
A = [5 0 -5 ; -11 -6 5 ; 5 0 -1]';
B = [1 0 1 ; -2 0 2]';

% using lab handout code to create Qc matrix
Qc = ctrb(A,B)

% fundamentals of Kalman's
if isequal(rank(Qc), size(A,2))
    disp('(A,B) is controllable')
else
    disp('(A,B) is not controllable')
end

% computing the image of Qc
V = orth(Qc);
V_rank = rank(V);

% contructing P matrix of representation theorem
V_kernel = null(transpose(V));
W = orth(V_kernel);

P = [V W];

% producing the controllable and uncontrollable subsystems
A_hat = inv(P)*A*P

B_hat = inv(P)*B
