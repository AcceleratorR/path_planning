
function Astar_path = Astar(Start,Goal,Map)
global map Gridsize thickenline;

if (nargin == 3)
    map = Map;
end

%gridsize 
Gridsize = 3;

%create neighboors matrix
Neighboors = create_around(Gridsize);

%% initialize

closelist = zeros(length(thickenline),4);% (point,father,G)

openlist = zeros(length(thickenline),5); % (point ,father,G,F)

count_openlist = 2;

count_closelist = 1;

openlist(:,5) = inf;


startH = cal_H(Start,Goal);

startF = startH ;

openlist(1,:) = [Start,0,0,startF];

% cal H in thickenline
thickenline(:,3) = cal_H(thickenline(:,1:2),Goal);

%% main loop
findpath =0;
 % when Goal is not in openlist or openlist is not empty
while(findpath == 0)
    
    if(openlist(1) == 0)
        error("cant find the path");
    end
  
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

    % point around
    around_point = currentpoint + Neighboors;
    %check point
    [~,row_l] = ismember(around_point,thickenline(:,1:2),'rows');
    
    row_l(row_l==0)=[];
    
    around_point = thickenline(row_l,:);
   
    N = size(around_point,1);
    
    % traverse point around
    for i = 1:1:N
        p = around_point(i,1:2);%[open_row,open_col];            
        % in closelist or in obstacle
        if (ismember(p,closelist(1:count_closelist-1,1:2),'rows') )           
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
            %openlist(currentpoint_row,4) + cal_G(p,currentpoint);
            openlist(count_openlist,4) =closelist(openlist(count_openlist,3),4)+cal_G(p,currentpoint);

            H = around_point(i,3);%cal_H(p,Goal);

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
            plot(p(2),p(1),'b.');
            hold on ;
            drawnow;
                
       end    
         % if findpath ,return
        if(findpath ==1)
            break;
        end
    end  
end

Astar_path = linkpath(closelist(:,1:3));
Astar_path = flip(Astar_path);
end
