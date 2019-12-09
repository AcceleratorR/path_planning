function  [pathfind,newnode] = extend_rrtree1only(tree2node)
global  map ;
pathfind = 0;
[map_row,map_col] = size(map);
while(pathfind == 0)
% rand num 
     if (rand()<0.5)
        rand_node = rand(1,2).*[map_row,map_col];   
     else
        rand_node = tree2node;  % from Goal to  Start
     end
     
    % extend tree1
     newnode = extendtree1(rand_node);
    % tree1stop
     [~,~,pathfind] = tree1stop(newnode);
end

end