% H distance
function hd = cal_H(a,b)
    hd = sum(abs(b-a),2);
end