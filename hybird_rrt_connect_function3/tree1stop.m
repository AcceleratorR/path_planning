function [nearest_node2,nearest_row2,pathfind] = tree1stop(newnode)
global rrtree2 tree2_num STEP thickenline Astar_Start
pathfind = 0;
%extend the second tree ,toward the newnode in the first tree
     [nearest_node2,nearest_row2] = find_nearest_node(newnode,rrtree2(1:tree2_num,1:2));
      
     %stop condition 1.1--tree1's newnode close to tree2 
     if (cal_H(nearest_node2,newnode) < STEP)
         pathfind = 3;
         newnode2 = nearest_node2;
         tree2_num = tree2_num+1;
         rrtree2(tree2_num,:) = [newnode2 nearest_row2];
         line([nearest_node2(2),newnode(2)],[nearest_node2(1),newnode(1)],'Color','g','LineWidth',2);
         return;
     end
     
     %stop condition -- tree1 close to thicken line
     [nearest_node_t,nearest_row_t] = find_nearest_node(newnode,thickenline);
     if(cal_H(nearest_node_t,newnode) < STEP)
        pathfind = 1;   
        Astar_Start = thickenline(nearest_row_t,:);
        %pathfind = extend_rrtree2only(newnode);
        return;
     end
     
end