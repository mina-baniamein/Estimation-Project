function Z_CdeltaD = zetagenCdeltaD(u, y, k, p)

z = zeros(1,2*p);
index = 1;

for i=k-p:k-1 
    z(1,2*index-1) = y(i);
    z(1,2*index) = u(i);
    index = index + 1;
end


Z_CdeltaD = flip(z);
end
