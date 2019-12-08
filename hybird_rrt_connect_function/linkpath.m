% link  path 
function finalpath = linkpath(rrtree)
 finalpath = zeros(length(rrtree),2);
 finalpath(1,:) =  rrtree(end,1:2);
 node_num = rrtree(end,3);
   for i =2:length(rrtree)
       if (node_num > 0)
       finalpath(i,:) = rrtree(node_num,1:2);
       node_num = rrtree(node_num,3);
       else
           finalpath(i,:) = rrtree(1,1:2);
           break;
       end
   end
finalpath(i+1:end,:) = [];

end