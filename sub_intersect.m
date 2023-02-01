%% Function 2:
% Function which returns a basis for the intersection of subspaces inputted

function C = sub_intersect(A,B)
    % concatenating the inputs based on the lab handout
    mx_int = [A -B];
    C = null(mx_int);
end