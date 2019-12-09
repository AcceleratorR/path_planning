function newnode2 = extendtree2(rand_node,nearest_node2,nearest_row2)
global rrtree2 tree2_num 
%extend the first tree ,toward the rand node 
if(nargin == 1)
     [nearest_node2,nearest_row2] = find_nearest_node(rand_node,rrtree2(1:tree2_num,1:2));
end
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
end