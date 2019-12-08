function  [tree2_num,pathfind] = extend_rrtree2only(tree2_num,tree1node)
global  rrtree2 thickenline map STEP Astar_Goal;
[map_row,map_col] = size(map);
while(1)
% rand num 
     if (rand()<0.5)
        rand_node = rand(1,2).*[map_row,map_col];   
     else
        rand_node = tree1node;  % from Goal to  Start
     end
     
 [nearest_node2,nearest_row2] = find_nearest_node(rand_node,rrtree2(1:tree2_num,1:2));
 newnode2 = extend_newpoint(rand_node,nearest_node2);
 
 if (~isempty(newnode2))   
        tree2_num = tree2_num+1;
        rrtree2(tree2_num,:) = [newnode2 nearest_row2]; 
        line([newnode2(2),nearest_node2(2)],[newnode2(1),nearest_node2(1)],'LineWidth',2);
        hold on;
        drawnow;
        else
         newnode2 = nearest_node2;

 end

     %stop condition  1.2 -- tree2's newnode close to tree1
     if (cal_Hdist(tree1node,newnode2) < STEP)
         pathfind = 1;
         line([newnode2(2),nearest_node(2)],[newnode2(1),nearest_node(1)],'Color','g','LineWidth',2);
         fprintf ('stop condition 1.2');
         break;
     end
     
     %stop condition -- tree2 close to thicken line
     [nearest_node_t,nearest_row_t] = find_nearest_node(newnode2,thickenline);
     if(cal_Hdist(nearest_node_t,newnode2) < STEP)
        pathfind = 2;
        Astar_Goal = thickenline(nearest_row_t,:);
        break;
     end
end
return;
end