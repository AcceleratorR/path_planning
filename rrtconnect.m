% function traject = rrtconnect(map,start,goal)


     % parameters
     clear all;clc;
    m = 5; %inflation parameters  recommend 6-7

    k = 2; %power of  curve fit   recommend 2-3

    num = 3;  %number of interp   recommend 2-3
     n = 100;
    mapp = zeros(n);
    center = n/2;
    block = center*3/4;
    mapp(center-block:center+block,center-block:center+block) = 1;
    map = ~repmat(mapp,5,5);
    
%     map = im2bw(imread('blank.jpg')); % input map
    
    originmap = map;

    start=[100 100]; 

    goal=[400 400]; % goal in Y, X 

    stepsize=10; 	% size of each step of the RRT

    disTh=stepsize; 	% nodes closer than this threshold are taken as almost the same

    maxFailedAttempts = 20000;

    display=false;

    iter(1)=0;         %iteration times

    iter(2) = checkblock(start,goal,originmap);  % checkblock

    RRTree1 = double([start -1]); % First RRT 

    RRTree2 = double([goal  -1]); % Second RRT 

    % counter=0;
    tree1ExpansionFail = false; % sets to true if expansion after set number of attempts fails

    tree2ExpansionFail = false; % sets to true if expansion after set number of attempts fails

    %tic;

      % inflation of map


     % check start and goal
    if ~feasiblePoint(start,map)
        error('source lies on an obstacle or outside map');
    end

    if ~feasiblePoint(goal,map)
        error('goal lies on an obstacle or outside map');
    end


     % main loop
    while ~tree1ExpansionFail || ~tree2ExpansionFail  % loop to grow RRTs      
        if ~tree1ExpansionFail
            [RRTree1,pathFound,tree1ExpansionFail,iter] = rrtExtend(RRTree1,RRTree2,goal,stepsize,maxFailedAttempts,disTh,map,iter); % RRT 1 expands from source towards goal  
            if ~tree1ExpansionFail && isempty(pathFound) && display
                line([RRTree1(end,2);RRTree1(RRTree1(end,3),2)],[RRTree1(end,1);RRTree1(RRTree1(end,3),1)],'color','b');
                counter=counter+1;
            end
        end
        if ~tree2ExpansionFail 
            [RRTree2,pathFound,tree2ExpansionFail,iter] = rrtExtend(RRTree2,RRTree1,start,stepsize,maxFailedAttempts,disTh,map,iter); % RRT 2 expands from goal towards source
           %iter2=iter2+1;
            if ~isempty(pathFound) && ~tree2ExpansionFail
                pathFound(3:4)=pathFound(4:-1:3); 
            end % path found
            if ~tree2ExpansionFail && isempty(pathFound) && display
                line([RRTree2(end,2);RRTree2(RRTree2(end,3),2)],[RRTree2(end,1);RRTree2(RRTree2(end,3),1)],'color','r');
                counter=counter+1;
            end
        end
        if ~isempty(pathFound) % path found
             if display
                line([RRTree1(pathFound(1,3),2);pathFound(1,2);RRTree2(pathFound(1,4),2)],[RRTree1(pathFound(1,3),1);pathFound(1,1);RRTree2(pathFound(1,4),1)],'color','green');
                counter=counter+1;
            end
            path=[pathFound(1,1:2)]; % compute path
            prev=pathFound(1,3);     % add nodes from RRT 1 first
            while prev > 0
                path=[RRTree1(prev,1:2);path];
                prev=RRTree1(prev,3);
            end
            prev=pathFound(1,4); % then add nodes from RRT 2
            while prev > 0
                path=[path;RRTree2(prev,1:2)];
                prev=RRTree2(prev,3);
            end
            break;
        end
    end



     % find path
    if size(pathFound,1)<=0, 
        error('no path found. maximum attempts reached'); 
    end

    pathLength=0;

    for i=1:length(path)-1 
        pathLength=pathLength+distanceCost(path(i,1:2),path(i+1,1:2));
    end

    %fprintf('processing time=%d \nPath Length=%d \n\n', toc,pathLength); 
    fprintf('processing time=%d \nPath Length=%d \n\n',pathLength); 
   
    
    
     % clip
    crowdpath = interpo(path,num); %interpolation

    simplepath=clip(crowdpath,map);% clip excess roads

    
    

     % NURBS for smooth
    simplepath2(:,[1 2])=simplepath(:,[2 1]);

    P =simplepath2';

    n = length(P)-1; 

    U = linspace(0, 1, n+k+2); 

    p_u = SplinePoint(n,k,U,P);

    finalpath = [[start(2),start(1)];p_u;[goal(2),goal(1)]];

    %line(finalpath(:,1), finalpath(:,2), 'Marker','.','LineStyle','-', 'Color',[.9 .3 .9]);
    
    
    
 % generate path sequence
    trajectstep = 0.5;

    interval =10; %  0.1s

    traject = pathsequence(finalpath,trajectstep,interval);

     % plot path
       figure(2)
        imshow(map);
        rectangle('position',[1 1 size(map)-1],'edgecolor','k');
        line(path(:,2),path(:,1),'Color','red');hold on;
        plot(goal(2),goal(1),'g*');
        plot(start(2),start(1),'r*');
        axis on; grid on;
% 
        figure(3)
        imshow(originmap);
        rectangle('position',[1 1 size(map)-1],'edgecolor','k');
        line(simplepath(:,2),simplepath(:,1));hold on ;
        plot(simplepath(:,2),simplepath(:,1),'*');
        plot(goal(2),goal(1),'g*');
        plot(start(2),start(1),'r*');
        axis on; grid on;

        figure(4)
        imshow(originmap);
        rectangle('position',[1 1 size(map)-1],'edgecolor','k');
        line(finalpath(:,1), finalpath(:,2), 'Color',[.9 .3 .9]);hold on;
        %plot(finalpath(:,1), finalpath(:,2),'.');
        plot(goal(2),goal(1),'g*');
        plot(start(2),start(1),'r*');
        axis on; grid on;

     
% end












%%

    %******************************************************** definition of function ********************************************************
    

                         
 %% combineMapLayers
function [costmap,originmap] = combineMapLayers(mapLayers)
    %combineMapLayers
    %   Combine map layers struct into a single vehicleCostmap.

    combinedMap = mapLayers.StationaryObstacles + mapLayers.RoadMarkings + ...
        mapLayers.ParkedCars;
    combinedMap = im2single(combinedMap);

    res = 0.5; % meters
    costmap = vehicleCostmap(combinedMap, 'CellSize', res);
    originmap = combinedMap;
end
%% inflate.m                                       
function newmap = inflate(map,m)
    [row,col] = find (map==0);
    for i = 1: length(row)
    x = row(i);
    y = col(i);
        if ((x-m)>1 && (x+m) < length(map) &&(y-m)>1 && (y+m) < length(map)) 
            for X= (x-m):(x+m)
                for Y = (y-m): (y+m)
                    map(X,Y)=0;
                end
            end
        end
    end
    newmap = map;
end


%% check block
function probability = checkblock(start,goal,map)
    x1=0.5*(goal(2)+start(2));
    y1=0.5*(goal(1)+start(1));
    a1=0.7*pdist2(start,goal);                 %³¤Öá
    b1=0.7*a1;                  %¶ÌÖá
    sita=0:0.01:2*pi;
    ra=rand(1,length(sita));
    fi1= atan2(goal(1)-start(1),goal(2)-start(2));  %y/x;
    xt=(x1+a1*cos(fi1)*cos(sita)-b1*sin(fi1)*sin(sita));
    yt=(y1+a1*sin(fi1)*cos(sita)+b1*cos(fi1)*sin(sita));
    r = ((xt-x1).^2+(yt-y1).^2).^0.5;%length
    fil2 = atan2(yt-y1,xt-x1); %theta
    xx = x1+r.*cos(fil2).*ra;
    yy = y1+r.*sin(fil2).*ra;
    % probability=0;
    point = [round(xx);round(yy)]';
    point(find(point(:,2)<=1),:)=[];
    point(find(point(:,2)>=size(map,2)),:)=[];
    point(find(point(:,1)<=1),:)=[];
    point(find(point(:,1)>=size(map,1)),:)=[];
    blocknum=0;
    for i = 1:length(point)
        if map(point(i,1),point(i,2))~=1
            blocknum=blocknum +1;
        end
    end
    probability = blocknum/i;
 end



%% rrtExtend.m
function [RRTree1,pathFound,extendFail,iter] = rrtExtend(RRTree1,RRTree2,goal,stepsize,maxFailedAttempts,disTh,map,iter)
    pathFound=[]; %if path found, returns new node connecting the two trees, index of the nodes in the two trees connected
    failedAttempts=0;
    extendFail = true;
    m = 6; %inflation parameters  recommend 6-7
    cantPoint=0;
while failedAttempts <= maxFailedAttempts
    %iter=iter+1;
    if rand < iter(2)
       %   miu=rand(1,2) .* size(map);
        %  Sigma=[1.0,0.5;.5,.3];
         % sample = mvnrnd(miu,Sigma,1);    
      sample = rand(1,2) .* size(map); % random sample
        iter(1)=iter(1)+1;
    else
        sample = goal; 	% sample taken as goal to bias tree generation to goal
        iter(1)=iter(1)+1;
    end

    [A, I] = min( distanceCost(RRTree1(:,1:2),sample) ,[],1); % find the minimum value of each column
    closestNode = RRTree1(I(1),:);
	
	% moving from qnearest an incremental distance in the direction of qrand
    theta = atan2((sample(1)-closestNode(1)),(sample(2)-closestNode(2)));  % direction to extend sample to produce new node
    newPoint = double(int32(closestNode(1:2) + stepsize * [sin(theta)  cos(theta)]));
    for degree = 0:pi/4:2*pi
        nerghboor = round(int32(newPoint(1:2) + m * [cos(degree)  sin(degree)]));
        if ~feasiblePoint(nerghboor,map)
            cantPoint =1;
            break;
        end
    end
    if (cantPoint ==1)
        cantPoint =0;
        continue;
    end
    if ~checkPath(closestNode(1:2), newPoint, map) % if extension of closest node in tree to the new point is feasible
        failedAttempts = failedAttempts + 1;
        continue;
    end

    [A, I2] = min( distanceCost(RRTree2(:,1:2),newPoint) ,[],1); % find closest in the second tree
    if distanceCost(RRTree2(I2(1),1:2),newPoint) < disTh        % if both trees are connected
        pathFound=[newPoint I(1) I2(1)];extendFail=false;
        break; 
    end 
    [A, I3] = min( distanceCost(RRTree1(:,1:2),newPoint) ,[],1); % check if new node is not already pre-existing in the tree
    if distanceCost(newPoint,RRTree1(I3(1),1:2)) < disTh, failedAttempts=failedAttempts+1;
        continue;
    end 
    RRTree1 = [RRTree1;newPoint I(1)];extendFail=false;
    break; % add node
end
end


%% checkPath.m	
function feasible=checkPath(nowpos,newPos,map,n)
    feasible=true;
    if nargin == 3
        step=0.5;
    else 
         step=0.5/n;
    end
    dir=atan2(newPos(1)-nowpos(1),newPos(2)-nowpos(2));
    for len=0:step:sqrt(sum((nowpos-newPos).^2))
        posCheck=nowpos+len.*[sin(dir) cos(dir)];
        %plot(posCheck(2),posCheck(1),'o');
        if ~(feasiblePoint(ceil(posCheck),map) && feasiblePoint(floor(posCheck),map) && ... 
                feasiblePoint([ceil(posCheck(1)) floor(posCheck(2))],map) && feasiblePoint([floor(posCheck(1)) ceil(posCheck(2))],map))
            feasible=false;
            break;
        end
        if ~feasiblePoint(newPos,map), feasible=false; end
    end
end


%% feasiblePoint.m
function feasible=feasiblePoint(point,map)
    feasible=true;
    % check if collission-free spot and inside maps
    if ~(point(1)>=1 && point(1)<=size(map,1) && point(2)>=1 && point(2)<=size(map,2) && map(point(1),point(2))==1)
        feasible=false;
    end
end

 %% interpo.m 
function newPath = interpo(path,num)
    newPath =path(1,:);
    node1=zeros(1,2);
    node2=zeros(1,2);
    for i = 1:length(path)-1
        node1 = path(i,:);
        node2 = path(i+1,:);
        for t= 1/num: 1/num :1
           newnode =  [1-t,t] * [node1;node2];
            newPath = [newPath;newnode];
        end
    end
    newPath = round(newPath);
   
end

%% clip.m
function simplepath=clip(path,map)
    threshold_theta = 50;
    [l,~]=size(path);
    simplepath=path(1:2,:);
    lastnode=path(2,:);
    dirr = simplepath(2,:) - simplepath(1,:);
    fixi = 2;
     for i = 1:l-1
         if ~checkPath(lastnode,path(i,:),map) 
            i = checkDegree(lastnode,path,dirr,i-1,fixi,threshold_theta);
             simplepath=[simplepath;path(i,:)];
             lastnode=path(i,:);
             fixi = i;
             dirr = simplepath(end,:) - simplepath(end-1,:);
         end
     end
    olddir = simplepath(end,:) - simplepath(end-1,:);
    newdir = path(end,:) - simplepath(end,:);
    angel = acos(dot(newdir,olddir)/(norm(olddir)*norm(newdir))) *180 /pi;
    if angel < threshold_theta
        simplepath=[simplepath;path(end,:)];
    else
        theta = atan2((path(end,1)-simplepath(end,1)),(path(end,2)-simplepath(end,2))); 
        newnode = double(int32(simplepath(end,:) + 20 * [sin(theta)  cos(theta)]));
        simplepath = [simplepath;newnode];
        simplepath=[simplepath;path(end,:)];
    end
end

 %% checkDegree
function ind = checkDegree(lastnode,nowpath,olddir,num,fixnum,threshold)
    newdir = nowpath(num,:) - lastnode;
    angel = acos(dot(newdir,olddir)/(norm(olddir)*norm(newdir))) *180 /pi;

    if (num) > fixnum
        if angel < threshold
        ind = num;
        else
        ind = checkDegree(lastnode,nowpath,olddir,num-1,fixnum,threshold);
        end
    else
        ind =fixnum+2;
    end

end


%% BaseFunction.m
function Nik_u = BaseFunction(i, k , u, U)
    if k == 0       
        if (u >= U(i+1)) && (u <U(i+2))
            Nik_u = 1.0;
        else
            Nik_u = 0.0;
        end
    else
        Length1 = U(i+k+1) - U(i+1);
        Length2 = U(i+k+2) - U(i+2);      
        if Length1 == 0.0       %define 0/0 = 0
            Length1 = 1.0;
        end
        if Length2 == 0.0
            Length2 = 1.0;
        end
        Nik_u = (u - U(i+1)) / Length1 * BaseFunction(i, k-1, u, U) ...
            + (U(i+k+2) - u) / Length2 * BaseFunction(i+1, k-1, u, U);
    end
end


%% SplinePoint.m  
function p_u = SplinePoint(n, k,U, P)
    Niku = zeros(n+1,1);
    p_u = [];
    for u = k/(n+k+1) : 0.01 : (n+1)/(n+k+1)
        for i = 0:n
            Niku(i+1,1) =  BaseFunction(i, k, u, U);
        end
        tmp = (P * Niku)';
        p_u = [p_u;tmp];
    %plot(p_u(1,1),p_u(2,1));

    end
end




%%  pathsequence.m
function traject = pathsequence(path,step,interval)

    traject = path;

    tmp1 = [];
    tmp2 = [];
    for n = 2:length(traject)
        tmp1=traject(n-1,:);
        now=traject(n-1,:);
        theta = atan2((traject(n,1)-traject(n-1,1)),(traject(n,2)-traject(n-1,2)));
        I = floor(distanceCost(traject(n-1,:),traject(n,:))/step); % distance
        for i=1:I
            newPoint = double(now(1:2) + step * [sin(theta)  cos(theta)]);
            if (distanceCost(newPoint,traject(n,:))<step)
                tmp1 = [tmp1;newPoint;traject(n,:)];
                tmp2 = [tmp2;tmp1];
                tmp1 = [];
                break;
            else
                tmp1=[tmp1;newPoint];
                now = newPoint;
            end
        end
    end
    traject = tmp2;
    if nargin ==3
        for i = 2:length(traject)
             traject(i,3)= distanceCost(traject(i-1,1:2),traject(i,1:2))/step/interval; %time
        end
      traject(1,3)=0;
      sum=0;
       for i = 1:length(traject)
           traject(i,3)=traject(i,3)+sum;
           sum = traject(i,3);
       end
    traject(:,[1 2 3])=traject(:,[3 1 2]);
    end
end

 %% distanceCost.m
function h=distanceCost(a,b)
	h = sqrt(sum((a-b).^2, 2));
end
	