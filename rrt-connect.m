clear all;
clc;

global map map_row map_col STEP Start Goal rrtree1 rrtree2;

%start and goal

Start = [240 240];

Goal = [2 2];

STEP = 20;

% % 1 map( blank =1,block =0 )

% n = 100;
% 
% mapp = zeros(n);
% 
% center = n/2;
% 
% block = center*2/4;
% 
% mapp(center-block:center+block,center-block:center+block) = 1;
% 
% map = ~repmat(mapp,5,5);
% 
% [map_row,map_col] = size(map);
% 
% clear block center mapp n;

% second map (input)
I = imread('xiaokou.jpg');
map = imbinarize(I);
map = map(:,:,1);

[map_row,map_col] = size(map);
clear I;

% init

if (iscollision(Start) == true)
    error("Start lies on an obstacle or outside map");
end

if (iscollision(Goal) == true)
    error("Goal lies on an obstacle or outside map");
end

MAXTIMES = 100000;

rrtree1 = zeros(MAXTIMES,3);
rrtree2 = zeros(MAXTIMES,3);

rrtree1(1,:) = [Start 0 ];  % node,father
rrtree2(1,:) = [Goal 0 ];

tree1_num =1 ;
tree2_num =1 ;

newnode1 =Start;
newnode2 =Goal;



run_times =0;

pathfind = 0;

figure(1)
imshow(map);
axis on;
hold on;
plot(Start(2),Start(1),'g.','Markersize',25);
plot(Goal(2),Goal(1),'b.','Markersize',25);

%% main loop
while (pathfind ==0  )
    %findpath =1;
    if (run_times >MAXTIMES)
        %findpath =0;
        error('cant find path');
    end
    % rand num 
     if (rand()<0.5)
        rand_node = rand(1,2).*[map_row,map_col];   
     else
           if (tree1_num <= tree2_num)
                rand_node = Goal;   % from Start to Goal
           else
                rand_node = Start;  % from Goal to  Start
           end   
     end
     run_times  = run_times+1;

     %extend rrtree
     %chose the tree which has less node extend first
     [tree1_num,tree2_num,pathfind] = rrtextend(rand_node,tree1_num,tree2_num); 
      
end

%%



%clear remain space
rrtree1(tree1_num+1:end,:)=[];
rrtree2(tree2_num+1:end,:)=[];
rrt1 = linkpath(rrtree1);
rrt2 = linkpath(rrtree2);
 
 rrt1 = flip(rrt1);
finalpath = [rrt1;rrt2];
%path length
length_path = sum(sum(diff(finalpath).^2,2).^0.5);
fprintf ('length path = %s \n',num2str(length_path));

figure(1)
plot(finalpath(:,2),finalpath(:,1),'r','LineWidth',2);


%%
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

 % rrtree extend
 function [tree1_num,tree2_num,pathfind] = rrtextend(rand_node,tree1_num,tree2_num)
 global rrtree1 rrtree2 STEP;
 pathfind = 0;
 
     
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
      
     %stop condition 1.1
     if (cal_Hdist(nearest_node2,newnode) < STEP)
         pathfind = 1;
         newnode2 = nearest_node2;
         tree2_num = tree2_num+1;
         rrtree2(tree2_num,:) = [newnode2 nearest_row2];
         line([nearest_node2(2),newnode(2)],[nearest_node2(1),newnode(1)],'Color','g','LineWidth',2);
         fprintf ('stop condition 1.1');
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
      
     %stop condition  1.2
     if (cal_Hdist(nearest_node,newnode2) < STEP)
         pathfind = 1;
         newnode = nearest_node;
         tree1_num = tree1_num+1;
         rrtree1(tree1_num,:) = [newnode nearest_row];
         line([newnode2(2),nearest_node(2)],[newnode2(1),nearest_node(1)],'Color','g','LineWidth',2);
         fprintf ('stop condition 1.2');
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
     
     %stop condition 2.1
     if (cal_Hdist(nearest_node,newnode2) < STEP)
         pathfind = 1;
         newnode = nearest_node;
         tree1_num = tree1_num+1;
         rrtree1(tree1_num,:) = [newnode nearest_row];
         line([nearest_node2(2),nearest_node(2)],[nearest_node2(1),nearest_node(1)],'Color','g','LineWidth',2);
         fprintf ('stop condition 2.1');
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
     
     %stop condition 2.2
     if (cal_Hdist(nearest_node2,newnode) < STEP)
         pathfind = 1;
         newnode2 = nearest_node2;
         tree2_num = tree2_num+1;
         rrtree2(tree2_num,:) = [newnode2 nearest_row2];
         line([newnode(2),nearest_node2(2)],[newnode(1),nearest_node2(1)],'Color','g','LineWidth',2);
         fprintf ('stop condition 2.2');
         return;
     end
     
 end

 end
 
% newnode 
function newnode = extend_newpoint(rand_node,nearest_node)
global STEP;
    theta = atan2((rand_node(1)-nearest_node(1)),(rand_node(2)-nearest_node(2)));  % direction to extend sample to produce new node
    
%     sline = linspace(0,STEP,30)';      % generete 10 points in line
%     detect_array = round(nearest_node + sline.* [sin(theta)  cos(theta)]);
%     detect_array = unique(detect_array,'rows','stable');    % generete points towards to rand_node
%     
%      for i = 1:length(detect_array)
%         if (iscollision(detect_array(i,:)) == true)
%             newnode = [];
%             return;
%         end    
%      end 
%     newnode = detect_array(end,:);
    newnode = round(nearest_node + STEP.* [sin(theta)  cos(theta)]);%detect_array(i,:);
    if (iscollision(newnode) == true)
        newnode = [];
        return;
    end
end

%find_nearest_node
function  [nearest_node,rowe] = find_nearest_node(rand_node,rrtree)
    dist_array = cal_dist(rand_node,rrtree);
    [~,row] = min(dist_array);
    rowe = row(end,:);
    nearest_node = rrtree(rowe,1:2);
    
end
%target


%iscollision
function  incollision = iscollision(p)
%( blank =1,block =0 )
global map map_row map_col;
    incollision = true;
    if (p(1)<=map_row && p(1)>=1 && p(2)<=map_col && p(2)>=1 && map(p(1),p(2))== 1)
    incollision =false;
    end

end

% distance
function d = cal_dist(a,b)
    d =  sum((a-b).^2,2).^0.5;
end

% H distance
function hd = cal_Hdist(a,b)
    hd = sum(abs(b-a))/1.1;
end