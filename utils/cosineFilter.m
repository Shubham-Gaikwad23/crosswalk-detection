function M = cosineFilter(width, height)

M = -1 * ones(1,width);
B = 2 * ones(1,width);

M = [M B M];

if height == 0
    height = (3*width);
end

M = repmat(M,height,1);

end