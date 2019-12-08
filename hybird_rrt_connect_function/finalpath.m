%Astar_path
function Astar_path = finalpath(closelist)
    [cr,~] = size(closelist);
    Astar_path = zeros(cr,2);
    i = 1;
    while(cr>0)
        Astar_path(i,:) = closelist(cr,1:2);
        cr = closelist(cr,3);
        i = i+1;
    end
    Astar_path(i:end,:) =[];
end