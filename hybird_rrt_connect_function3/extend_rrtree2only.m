function  [pathfind,newnode2] = extend_rrtree2only(tree1node)
global  map ;
pathfind = 0;
[map_row,map_col] = size(map);
while(pathfind == 0)
% rand num 
     if (rand()<0.5)
        rand_node = rand(1,2).*[map_row,map_col];   
     else
        rand_node = tree1node;  % from Goal to  Start
     end
     
    newnode2 = extendtree2(rand_node);

      % tree2 stop condition 
    [~,~,pathfind] = tree2stop(newnode2);
    
end

end