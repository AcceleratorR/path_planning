function [nearest_node,nearest_row,pathfind] = tree2stop(newnode2)
global rrtree1 tree1_num STEP thickenline Astar_Goal
pathfind = 0;
%extend the second tree ,toward the newnode in the first tree
     [nearest_node,nearest_row] = find_nearest_node(newnode2,rrtree1(1:tree1_num,1:2));
      
     %stop condition 1.1--tree1's newnode close to tree2 
     if (cal_H(nearest_node,newnode2) < STEP)
         pathfind = 3;
         newnode2 = nearest_node;
         tree1_num = tree1_num+1;
         rrtree1(tree1_num,:) = [newnode2 nearest_row];
         line([nearest_node(2),newnode2(2)],[nearest_node(1),newnode2(1)],'Color','g','LineWidth',2);
         return;
     end
     
     %stop condition -- tree1 close to thicken line
     [nearest_node_t,nearest_row_t] = find_nearest_node(newnode2,thickenline);
     if(cal_H(nearest_node_t,newnode2) < STEP)
        pathfind = 2;   
        Astar_Goal = thickenline(nearest_row_t,:);
        %pathfind = extend_rrtree2only(newnode);
        return;
     end
     
end