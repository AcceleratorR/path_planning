 % rrtree extend
 function pathfind = rrtextend(rand_node)
 global  tree1_num tree2_num;
 %pathfind = 1 -->tree1-tree2
 %pathfind = 2 -->tree1/2-thickenline

     
 if (tree1_num <= tree2_num) 
     % extend tree1
     newnode = extendtree1(rand_node);
     
     % tree stop condition
    [nearest_node2,nearest_row2,pathfind] = tree1stop(newnode);
     if(pathfind>0)
         return;
     end
     
     %extend tree2
     newnode2 = extendtree2(newnode,nearest_node2,nearest_row2);
     
      % tree2 stop condition 
    [~,~,pathfind] = tree2stop(newnode2);  
     
 else   

      %  extend tree2
     newnode2 = extendtree2(rand_node);
     
     % tree2 stop condition 
    [nearest_node,nearest_row,pathfind] = tree2stop(newnode2);
     if(pathfind>0)
         return;
     end
     
     %extend tree1
     newnode = extendtree1(newnode2,nearest_node,nearest_row);
     
    % tree stop condition
    [~,~,pathfind] = tree1stop(newnode);
     
 end

 end