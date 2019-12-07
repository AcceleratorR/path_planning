clear all;
clc;

global map map_row map_col Gridsize ;

% change start , goal and gridsize 
Start = [200 200];

Goal = [50 50];

Gridsize =10;

% map( blank =1,block =0 )
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

clear I;

%% initialize

closelist = zeros(round(map_row*map_col/3),4);% (point,father,G)

openlist = zeros(round(map_row*map_col/3),5); % (point ,father,G,F)

count_openlist = 2;

count_closelist = 1;

openlist(:,5) = inf;


if (iscollision(Start) == true)
    error("Start lies on an obstacle or outside map");
end

if (iscollision(Goal)  == true)
    error("Goal lies on an obstacle or outside map");
end

startH = cal_H(Start,Goal);

startF = startH ;

openlist(1,:) = [Start,0,0,startF];

currentpoint = Start;

%figure
figure(1)

imshow(map);

axis on;

hold on;

plot(Start(2),Start(1),'k.','Markersize',15);

plot(Goal(2),Goal(1),'b.','Markersize',15);

%% main loop
findpath =0;
 % when Goal is not in openlist or openlist is not empty
while(findpath == 0)
    
    if(openlist(1) == 0)
        error("cant find the path");
    end
    
    % (point father G F)
    
    %find min F in openlist
    [~,currentp_row] = min(openlist(1:count_openlist,5)); 
    currentpoint_row = currentp_row(end);
    
    % set it for current point 
    currentpoint = openlist(currentpoint_row,1:2) ; 
    
    %put current point in closelist,count_closelist +1
    closelist(count_closelist,:) = openlist(currentpoint_row,1:4); 
    
    count_closelist = count_closelist +1;
    
    openlist(currentpoint_row,:) = [];
    
    count_openlist = count_openlist -1;
    
    % traverse point around
    for open_row = currentpoint(1) - Gridsize : Gridsize: currentpoint(1) + Gridsize

        for open_col = currentpoint(2) - Gridsize :  Gridsize: currentpoint(2) + Gridsize 
           
            p = [open_row,open_col];   
            
            % in closelist or in obstacle
            if (iscollision(p) || ismember(p,closelist(1:count_closelist-1,1:2),'rows') )           
                continue; 
            end
            
            [ismem,mem_row] = ismember(p,openlist(1:count_openlist,1:2),'rows');
            
           if(ismem ==1 )% p already in openlist
               
               %cal new G
               currentG = closelist(count_closelist-1,4) + cal_G(p,currentpoint);
              
               % from current point to this point v.s origin path by G 
               if(currentG < openlist(mem_row,4)) 
                   
                   %change F
                   openlist(mem_row,5) =  openlist(mem_row,5) + currentG - openlist(mem_row,4);
                   
                   %change G
                   openlist(mem_row,4) = currentG;
                   
                   %change father
                   openlist(mem_row,3) = count_closelist-1;
               end
                         
            else   % p is not in openlist 
                
                % put p in openlist
                openlist(count_openlist,1:2) = p;
                
                % set father
                openlist(count_openlist,3) = count_closelist-1;  
                
                % cal G H F
                openlist(count_openlist,4) =closelist(openlist(count_openlist,3),4)+cal_G(p,currentpoint);%openlist(currentpoint_row,4) + cal_G(p,currentpoint);
                
                H = cal_H(p,Goal);
               
                openlist(count_openlist,5) = openlist(count_openlist,4)+ H;
                
                % find path
                if(cal_H(openlist(count_openlist,1:2),Goal) < Gridsize)
                   
                    closelist(count_closelist,:) = openlist(count_openlist,1:4);
                    
                    findpath = 1;
                    
                    %clear remain space
                    closelist(count_closelist+1:end,:) = [];
                    
                    openlist(count_openlist+1:end,:) = [];
                    break;
                end
                
                % openlist count +1
                count_openlist = count_openlist + 1;
                
                % figure
                figure(1)
                plot(p(2),p(1),'g.');
                hold on ;
                drawnow;
                
            end
        end
        
         % if findpath ,return
        if(findpath ==1)
            break;
        end
        
    end
end

Astar_path = finalpath(closelist(:,1:3));

%figure
figure(1)
plot(Astar_path(:,2),Astar_path(:,1),'r','LineWidth',2);

%% 
%Astar_path
function Astar_path = finalpath(closelist)
    [cr,~] = size(closelist);
    Astar_path = zeros(cr,2);
    i = 1;
    while(cr>0)
        Astar_path(i,:) = closelist(cr,1:2);
        cr = closelist(cr,3);
        i = i+1;
    end
    Astar_path(i:end,:) =[];
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

% G distance

function d = cal_G(a,b)
global Gridsize;
   if ( (abs(a(1) - b(1)) == Gridsize) && (abs(a(2) - b(2)) == Gridsize ) )
     d = 1.4*Gridsize;
   else
     d = 1*Gridsize;
    end
end

% distance
function d = cal_dis(a,b)
    d =  sum((a-b).^2,2).^0.5;
end

% H distance
function hd = cal_H(a,b)
    hd = sum(abs(b-a));
end
