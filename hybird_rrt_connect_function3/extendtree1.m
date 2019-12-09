function newnode = extendtree1(rand_node,nearest_node,nearest_row)
global rrtree1 tree1_num 
%extend the first tree ,toward the rand node 
if(nargin == 1)
     [nearest_node,nearest_row] = find_nearest_node(rand_node,rrtree1(1:tree1_num,1:2));
end
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
     
     
end