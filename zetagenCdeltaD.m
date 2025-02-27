function Z_CdeltaD = zetagenCdeltaD(u, y, k, p)
    % Validate inputs
    if length(u) ~= length(y)
        error('Vectors u and y must have the same length.');
    end
    
    % Construct z based on the given pattern
    z = reshape([y(:) u(:)]', [], 1); % Interleave u and y into a single column vector
    
    % Validate index range
    if k < 1 || k + p - 1 > length(z)
        error('Invalid indices: Ensure k and p are within the range of constructed z.');
    end
    
    % Extract the required segment and flip it to match the equation
    Z_CdeltaD = flip(z(k-p:k-1))';
end