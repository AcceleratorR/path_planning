%iscollision
function  incollision = iscollision(p)
%( blank =1,block =0 )    
global map ;
p = round(p);
    [map_row,map_col] = size(map);
    incollision = false;
    if ~(p(1) <= map_row && p(1)>=1 && p(2)<=map_col && p(2)>=1 && map(p(1),p(2))== 1)
        incollision =true;
    end
end