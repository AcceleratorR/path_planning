% H distance
function hd = cal_Hdist(a,b)
    hd = sum(abs(b-a))/1.1;
end