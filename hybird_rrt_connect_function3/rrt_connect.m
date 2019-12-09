function finalpath =  rrt_connect(Start,Goal,Map)
global map STEP rrtree1 rrtree2 Astar_Start Astar_Goal tree1_num tree2_num;
if (nargin == 3)
    map = Map;
end


STEP = 20;

[map_row,map_col] = size(map);

MAXTIMES = 100000;

rrtree1 = zeros(MAXTIMES,3);
rrtree2 = zeros(MAXTIMES,3);

rrtree1(1,:) = [Start 0 ];  % node,father
rrtree2(1,:) = [Goal 0 ];

tree1_num =1 ;
tree2_num =1 ;

% newnode1 =Start;
% newnode2 =Goal;

run_times =0;

pathfind = 0;

%% main loop
while (pathfind ==0 )
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
     pathfind = rrtextend(rand_node); 
end

%%
   % if find path ==3
    %clear remain space
if(pathfind == 3)  
    Astar_path = [];
    
elseif(pathfind == 1)  %find tree1 
    extend_rrtree2only(rrtree1(tree1_num,1:2)); 
    Astar_path = Astar(Astar_Start,Astar_Goal);
    
elseif(pathfind == 2)  %find tree1 
    extend_rrtree1only(rrtree2(tree2_num,1:2)); 
    Astar_path = Astar(Astar_Start,Astar_Goal);
end  
    rrtree1(tree1_num+1:end,:)=[];
    rrtree2(tree2_num+1:end,:)=[];
    rrtree1 = linkpath(rrtree1);
    rrtree2 = linkpath(rrtree2);
    rrtree1 = flip(rrtree1);
    finalpath = [rrtree1;Astar_path;rrtree2];
end