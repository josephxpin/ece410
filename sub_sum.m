%% Function 1:
% Function which returns a basis for the sum of subspaces inputted

function C = sub_sum(A, B)
    mx_int = A + B;
    C = orth(mx_int);
end