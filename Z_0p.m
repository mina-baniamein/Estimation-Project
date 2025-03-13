function Z_vect = Z_0p(u, y, k, p)

%   DIAG GAMMADELTA
%
%

index = 1 ;

% Construction of Z_vect 
for i= k : k + p - 1
    if i == 0
        Z_vect(2*index-1,1) = 0;
        Z_vect(2*index,1) = 0;
    else
        Z_vect(2*index-1,1) = u(i);
        Z_vect(2*index,1) = y(i);
    end
    index = index + 1;
end

end