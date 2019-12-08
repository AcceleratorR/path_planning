% rrtree extend
 function [tree1_num,tree2_num,pathfind] = rrtextend(rand_node,tree1_num,tree2_num)
 global rrtree1 rrtree2 thickenline STEP Astar_Goal Astar_Start;
 pathfind = 0;
 %pathfind = 1 -->tree1-tree2
 %pathfind = 2 -->tree1-thickenline
 %pathfind = 3 -->tree2-thickenline
     
 if (tree1_num <= tree2_num)
     
    
     % tree1_num < tree2_num
     %extend the first tree ,toward the rand node 
     [nearest_node,nearest_row] = find_nearest_node(rand_node,rrtree1(1:tree1_num,1:2));
     
     newnode = extend_newpoint(rand_node,nearest_node);
     
     %if get new node ,extend  first  tree
     if (~isempty(newnode))   
        tree1_num = tree1_num+1;
        rrtree1(tree1_num,:) = [newnode nearest_row];
        line([newnode(2),nearest_node(2)],[newnode(1),nearest_node(1)],'Color','magenta','LineWidth',2);
        hold on;
        drawnow;
     else
         newnode = nearest_node;
     end
     
     %extend the second tree ,toward the newnode in the first tree
     [nearest_node2,nearest_row2] = find_nearest_node(newnode,rrtree2(1:tree2_num,1:2));
      
     %stop condition 1.1--tree1's newnode close to tree2 
     if (cal_Hdist(nearest_node2,newnode) < STEP)
         pathfind = 1;
         newnode2 = nearest_node2;
         tree2_num = tree2_num+1;
         rrtree2(tree2_num,:) = [newnode2 nearest_row2];
         line([nearest_node2(2),newnode(2)],[nearest_node2(1),newnode(1)],'Color','g','LineWidth',2);
         fprintf ('stop condition 1.1');
         return;
     end
     
     %stop condition -- tree1 close to thicken line
     [nearest_node_t,nearest_row_t] = find_nearest_node(newnode,thickenline);
     if(cal_Hdist(nearest_node_t,newnode) < STEP)
        Astar_Start = thickenline(nearest_row_t,:);
        [tree2_num,pathfind] = extend_rrtree2only(tree2_num,newnode);
        return;
     end
     
     newnode2 = extend_newpoint(newnode,nearest_node2);
     
     if (~isempty(newnode2))   
        tree2_num = tree2_num+1;
        rrtree2(tree2_num,:) = [newnode2 nearest_row2]; 
        line([newnode2(2),nearest_node2(2)],[newnode2(1),nearest_node2(1)],'LineWidth',2);
        hold on;
        drawnow;
        else
         newnode2 = nearest_node2;

     end
     
     [nearest_node,nearest_row] = find_nearest_node(newnode2,rrtree1(1:tree1_num,1:2));
      
     %stop condition  1.2 -- tree2's newnode close to tree1
     if (cal_Hdist(nearest_node,newnode2) < STEP)
         pathfind = 1;
         newnode = nearest_node;
         tree1_num = tree1_num+1;
         rrtree1(tree1_num,:) = [newnode nearest_row];
         line([newnode2(2),nearest_node(2)],[newnode2(1),nearest_node(1)],'Color','g','LineWidth',2);
         fprintf ('stop condition 1.2');
         return;
     end
     
    %stop condition -- tree2 close to thicken line
     [nearest_node_t,nearest_row_t] = find_nearest_node(newnode2,thickenline);
     if(cal_Hdist(nearest_node_t,newnode2) < STEP)
        pathfind = 2;
        Astar_Goal = thickenline(nearest_row_t,:);
        [tree1_num,pathfind] = extend_rrtree1only(tree1_num,newnode2);
        return;
     end
     
     
 else   %  following is the condition tree1_num > tree2_num,same as forward
     
     
     
      %  tree1_num > tree2_num
     %extend the first tree ,toward the rand node 
     [nearest_node2,nearest_row2] = find_nearest_node(rand_node,rrtree2(1:tree2_num,1:2));
     
     newnode2 = extend_newpoint(rand_node,nearest_node2);
     
     %if get new node ,extend  first  tree
     if (~isempty(newnode2))   
        tree2_num = tree2_num+1;
        rrtree2(tree2_num,:) = [newnode2 nearest_row2];
        line([newnode2(2),nearest_node2(2)],[newnode2(1),nearest_node2(1)],'LineWidth',2);
        hold on;
        drawnow;
     else
         newnode2 = nearest_node2;

     end
     
     %extend the second tree ,toward the newnode in the first tree
     [nearest_node,nearest_row] = find_nearest_node(newnode2,rrtree1(1:tree1_num,1:2));
     
     %stop condition 2.1 -- tree2's newnode close to tree1
     if (cal_Hdist(nearest_node,newnode2) < STEP)
         pathfind = 1;
         newnode = nearest_node;
         tree1_num = tree1_num+1;
         rrtree1(tree1_num,:) = [newnode nearest_row];
         line([nearest_node2(2),nearest_node(2)],[nearest_node2(1),nearest_node(1)],'Color','g','LineWidth',2);
         fprintf ('stop condition 2.1');
         return;
     end
     
     %stop condition -- tree2 close to thicken line
     [nearest_node_t,nearest_row_t] = find_nearest_node(newnode2,thickenline);
     if(cal_Hdist(nearest_node_t,newnode2) < STEP)
        pathfind = 2;
        Astar_Goal = thickenline(nearest_row_t,:);
        [tree1_num,pathfind] = extend_rrtree1only(tree1_num,newnode2);
        return;
     end
     
     newnode = extend_newpoint(nearest_node2,nearest_node);
     
     if (~isempty(newnode))   
        tree1_num = tree1_num+1;
        rrtree1(tree1_num,:) = [newnode nearest_row]; 
        line([newnode(2),nearest_node(2)],[newnode(1),nearest_node(1)],'Color','magenta','LineWidth',2);
        hold on;
        drawnow;
        else
         newnode = nearest_node;
     end
     
     
     [nearest_node2,nearest_row2] = find_nearest_node(newnode,rrtree2(1:tree2_num,1:2));
     
     %stop condition 2.2 -- tree1's newnode close to tree2
     if (cal_Hdist(nearest_node2,newnode) < STEP)
         pathfind = 1;
         newnode2 = nearest_node2;
         tree2_num = tree2_num+1;
         rrtree2(tree2_num,:) = [newnode2 nearest_row2];
         line([newnode(2),nearest_node2(2)],[newnode(1),nearest_node2(1)],'Color','g','LineWidth',2);
         fprintf ('stop condition 2.2');
         return;
     end
     
     %stop condition -- tree1 close to thicken line
     [nearest_node_t,nearest_row_t] = find_nearest_node(newnode,thickenline);
     if(cal_Hdist(nearest_node_t,newnode) < STEP)
        pathfind = 2;
        Astar_Start = thickenline(nearest_row_t,:);
        [tree2_num,pathfind] = extend_rrtree2only(tree2_num,newnode);
        return;
     end
     
 end

 end