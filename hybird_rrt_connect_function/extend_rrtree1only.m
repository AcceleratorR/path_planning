function  [tree1_num,pathfind] = extend_rrtree1only(tree1_num,tree1node)
global  rrtree1 thickenline map  Astar_Start;
[map_row,map_col] = size(map);
while(1)
% rand num 
     if (rand()<0.5)
        rand_node = rand(1,2).*[map_row,map_col];   
     else
        rand_node = tree1node;  % from Goal to  Start
     end
     
 [nearest_node,nearest_row] = find_nearest_node(rand_node,rrtree1(1:tree1_num,1:2));
 newnode = extend_newpoint(rand_node,nearest_node);
 
 if (~isempty(newnode))   
        tree1_num = tree1_num+1;
        rrtree1(tree1_num,:) = [newnode nearest_row]; 
        line([newnode(2),nearest_node(2)],[newnode(1),nearest_node(1)],'LineWidth',2);
        hold on;
        drawnow;
        else
         newnode = nearest_node;

 end

     %stop condition  1.2 -- tree2's newnode close to tree1
     if (cal_Hdist(tree1node,newnode) < STEP)
         pathfind = 1;
         line([newnode(2),nearest_node(2)],[newnode(1),nearest_node(1)],'Color','g','LineWidth',2);
         fprintf ('stop condition 1.2');
         break;
     end
     
     %stop condition -- tree2 close to thicken line
     [nearest_node_t,nearest_row_t] = find_nearest_node(newnode,thickenline);
     if(cal_Hdist(nearest_node_t,newnode) < STEP)
        pathfind = 2;
        Astar_Start = thickenline(nearest_row_t,:);
        break;
     end
end
return;
end