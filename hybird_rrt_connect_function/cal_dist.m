% distance
function d = cal_dist(a,b)
    d =  sum((a-b).^2,2).^0.5;
end