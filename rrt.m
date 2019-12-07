clear all;
clc;

global map map_row map_col STEP ;

%start and goal
Start = [200 200];

Goal = [50 50];

% % map( blank =1,block =0 )
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

I = imread('xiaokou.jpg');
map = imbinarize(I);
map = map(:,:,1);

[map_row,map_col] = size(map);

% ³õÊ¼»¯

if (iscollision(Start) == true)
    error("Start lies on an obstacle or outside map");
end

if (iscollision(Goal) == true)
    error("Goal lies on an obstacle or outside map");
end

MAXTIMES = 100000;

rrtree = zeros(MAXTIMES,3);

rrtree(1,:) = [Start 0 ];  % node,father

tree_num =1 ;

newnode =Start;

STEP = ceil(min(map_row,map_col)/30);

run_times =0;

figure(1)
imshow(map);
axis on;
hold on;
plot(Start(2),Start(1),'g.','Markersize',25);
plot(Goal(2),Goal(1),'b.','Markersize',25);

%% main loop
while (cal_dist(rrtree(tree_num,1:2),Goal) > STEP )
    %findpath =1;
    if (run_times >MAXTIMES)
        %findpath =0;
        error('cant find path');
    end
    % rand num 
     if (rand()<0.49)
        rand_node = rand(1,2).*[map_row,map_col];   
     else
        rand_node = Goal;
     end
     run_times  = run_times+1;
     
     % obstacle detection
     [nearest_node,nearest_row] = find_nearest_node(rand_node,rrtree(1:tree_num,1:2));
     
     newnode = find_newpoint(rand_node,nearest_node);
     if (isempty(newnode))   
         continue;
     end
     
     line([newnode(2),nearest_node(2)],[newnode(1),nearest_node(1)],'LineWidth',2);
     hold on ;
     drawnow ; 
    tree_num = tree_num+1;
    rrtree(tree_num,:) = [newnode nearest_row];
end
  
rrtree(tree_num+1:end,:) = [];

% link  path 

 node_num = rrtree(end,3);
 finalpath = zeros(length(rrtree),2);
 finalpath(1,:) = Goal;
 finalpath(2,:) = rrtree(end,1:2);

   for i =3:length(rrtree)
       if (node_num > 0)
       finalpath(i,:) = rrtree(node_num,1:2);
       node_num = rrtree(node_num,3);
       else
           finalpath(i,:) = Start;
           break;
       end
   end
finalpath(i+1:end,:) = [];


%path length
length_path = sum(sum(diff(finalpath).^2,2).^0.5);
fprintf ('length path = %s \n',num2str(length_path));

figure(1)
plot(finalpath(:,2),finalpath(:,1),'r','LineWidth',2);


%%

function newnode = find_newpoint(rand_node,nearest_node)
global STEP;
    theta = atan2((rand_node(1)-nearest_node(1)),(rand_node(2)-nearest_node(2)));  % direction to extend sample to produce new node
%     sline = linspace(0,STEP,30)';      % generete 10 points in line
%     detect_array = round(nearest_node + sline.* [sin(theta)  cos(theta)]);
%     detect_array = unique(detect_array,'rows');    % generete points towards to rand_node
%     newnode = [];
%     for i = 1:length(detect_array)
%         if (iscollision(detect_array(i,:)) == true)
%             return;
%         end    
%     end 
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
