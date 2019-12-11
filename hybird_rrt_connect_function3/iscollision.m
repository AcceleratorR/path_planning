%iscollision
function  incollision = iscollision(p,Map)
%( blank =1,block =0 )    
global map ;
if(nargin == 1)
    Map = map;
end    

    [map_row,map_col] = size(Map);
    incollision = false;
    if ~(p(1) <= map_row && p(1)>=1 && p(2)<=map_col && p(2)>=1 && Map(p(1),p(2))== 1)
        incollision =true;
    end
end