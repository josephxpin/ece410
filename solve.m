%% Function 2:
% function which indicates the existence and uniqueness of a solution
% to the equation Ax = b

function x = solve(A,b)
    % flow statements will first determine existence, then uniqueness
    if isequal(rank(A),rank([A b]))
        if isequal(rank(A),size(A,2))
            disp('There is one solution to the equation Ax = b')
            x = A\b
        else
            disp('There are infinite solutions to the equation Ax = b')
            x = A\b
            x = pinv(A)*b
        end
    else
        disp('There are no solutions to the equation Ax = b')
        disp('Setting x = 0 to exit the function gracefully')
        x = 0
    end
end