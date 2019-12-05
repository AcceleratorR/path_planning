clc;
clear all;
close all;
  %% parameters
m =6; %inflation parameters  recommend 6-7
k = 3; %power of  curve fit   recommend 2-3
num = 3;  %number of inter   recommend 2-3

 originmap=im2bw(imread('ao.png')); % input map
%      n = 100;
%     mapp = zeros(n);
%     center = n/2;
%     block = center*3/4;
%     mapp(center-block:center+block,center-block:center+block) = 1;
%     originmap = ~repmat(mapp,5,5);
start=[200 900]; 
goal=[200 200]; % goal in Y, X 
stepsize=20; 	% size of each step of the RRT
disTh=20; 		% nodes closer than this threshold are taken as almost the same
maxFailedAttempts = 20000;
display=false; 	
iter=0;         %iteration times

RRTree1 = double([start -1]); % First RRT 
RRTree2 = double([goal  -1]); % Second RRT 

% counter=0;
tree1ExpansionFail = false; % sets to true if expansion after set number of attempts fails
tree2ExpansionFail = false; % sets to true if expansion after set number of attempts fails
tic;
  %% inflation of map
map=inflate(originmap,m);
%map = originmap;
if ~feasiblePoint(start,map)
    error('source lies on an obstacle or outside map');
end

if ~feasiblePoint(goal,map)
    error('goal lies on an obstacle or outside map');
end

if display 
    imshow(map);
    rectangle('position',[1 1 size(map)-1],'edgecolor','k');
end
 %% main loop
while ~tree1ExpansionFail || ~tree2ExpansionFail  % loop to grow RRTs      
    if ~tree1ExpansionFail
        [RRTree1,pathFound,tree1ExpansionFail,iter] = rrtExtend(RRTree1,RRTree2,goal,stepsize,maxFailedAttempts,disTh,map,iter); % RRT 1 expands from source towards goal  
        if ~tree1ExpansionFail && isempty(pathFound) && display
            line([RRTree1(end,2);RRTree1(RRTree1(end,3),2)],[RRTree1(end,1);RRTree1(RRTree1(end,3),1)],'color','b');
            counter=counter+1; %M(counter)=getframe;
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
            counter=counter+1;%M(counter)=getframe;
        end
    end
    if ~isempty(pathFound) % path found
         if display
            line([RRTree1(pathFound(1,3),2);pathFound(1,2);RRTree2(pathFound(1,4),2)],[RRTree1(pathFound(1,3),1);pathFound(1,1);RRTree2(pathFound(1,4),1)],'color','green');
            counter=counter+1;%M(counter)=getframe;
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
 %% find path
if size(pathFound,1)<=0
    error('no path found. maximum attempts reached'); 
end
pathLength=0;
for i=1:length(path)-1 
    pathLength=pathLength+distanceCost(path(i,1:2),path(i+1,1:2));
end
fprintf('processing time=%d \nPath Length=%d \n\n', toc,pathLength); 

 %% jianzhi

crowdpath = interpo(path,num); %cha zhi 
simplepath=jianzhi(crowdpath,map);

  %% NURBS
simplepath2(:,[1 2])=simplepath(:,[2 1]);
P =simplepath2';
n = length(P)-1; 
U = linspace(0, 1, n+k+2); % 均匀B样条的节点矢�?
p_u = SplinePoint(n,k,U,P);
finalpath = [[start(2),start(1)];p_u;[goal(2),goal(1)]];
%line(finalpath(:,1), finalpath(:,2), 'Marker','.','LineStyle','-', 'Color',[.9 .3 .9]);
 %% plot
figure(2)
imshow(map);
rectangle('position',[1 1 size(map)-1],'edgecolor','k');
line(path(:,2),path(:,1),'Color','red');hold on;
plot(goal(2),goal(1),'g*');
plot(start(2),start(1),'r*');
axis on; grid on;

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
plot(finalpath(:,1), finalpath(:,2),'.');
plot(goal(2),goal(1),'g*');
plot(start(2),start(1),'r*');
axis on; grid on;
%%












%% inflate.m
%%%%%%%%%%%%%%%%%%%%%%%%%%% definition of function %%%%%%%%%%%%%%%%%%%%%%%%%%%
function newmap = inflate(map,m)
[len,wid] = size(map);
[row,col] = find (map==0);
newinflate = zeros(length(row),3);
newinflate(:,3)= 0;

for i = 1: length(row)
x = row(i);
y = col(i);
if ((x-m)>1 && (x+m) < len &&(y-m)>1 && (y+m) < wid) 
    for X= (x-m):(x+m)
        for Y = (y-m): (y+m)
             
            if(map(X,Y)==1  )
               map(X,Y)=0;
               newinflate(i,1:2) = [X,Y];
               newinflate(i,3) =newinflate(i,3) +1;
            end

        end
    end
end
end

newinflate(newinflate(:,1)==0,:)=[];

newmap = map;
imshow(newmap);
hold on;
plot(newinflate(:,2),newinflate(:,1),'k.');
x=find(newinflate(:,3)>m);
hold on;
a=1;
while(a <= length(x))
     for i= 1:1:length(x)-a
         newinflate(x(a+i),1:2)
         newinflate(x(a),1:2)
        dis = distanceCost(newinflate(x(a+i),1:2),newinflate(x(a),1:2))
        if(dis<m)  
         x(a+i)=0;
        end
     end
    x(x==0)=[];
    a=a+1;
end   

    plot(newinflate(x,2),newinflate(x,1),'r.');
    hold on ;
    narrowpoint=zeros(length(x)*m,2);
    c =1 ;
 

for i =  1:length(x)
    for j =i:1:length(x)
    midx = floor(0.5*newinflate(x(i),1)+ 0.5*newinflate(x(j),1));
    midy = floor(0.5*newinflate(x(i),2)+ 0.5*newinflate(x(j),2));
    if(map(midx,midy) ==1)
        narrowpoint(c,:)=[midx,midy];
        c=c+1;
    end
    end
end
narrowpoint=unique(narrowpoint,'rows');

narrowpoint(narrowpoint(:,1)==0,:)=[];
    
plot(narrowpoint(:,2),narrowpoint(:,1),'bo');
end

%% rrtExtend.m
function [RRTree1,pathFound,extendFail,iter] = rrtExtend(RRTree1,RRTree2,goal,stepsize,maxFailedAttempts,disTh,map,iter)
pathFound=[]; %if path found, returns new node connecting the two trees, index of the nodes in the two trees connected
failedAttempts=0;
extendFail = true;
while failedAttempts <= maxFailedAttempts
    %iter=iter+1;
    if rand < 0.5 
        sample = rand(1,2) .* size(map); % random sample
        iter=iter+1;
    else
        sample = goal; 	% sample taken as goal to bias tree generation to goal
        iter=iter+1;
    end

    [A, I] = min( distanceCost(RRTree1(:,1:2),sample) ,[],1); % find the minimum value of each column
    closestNode = RRTree1(I(1),:);
	
	% moving from qnearest an incremental distance in the direction of qrand
    theta = atan2((sample(1)-closestNode(1)),(sample(2)-closestNode(2)));  % direction to extend sample to produce new node
    newPoint = double(int32(closestNode(1:2) + stepsize * [sin(theta)  cos(theta)]));
    
        
%     if ~checkNeighboor(newPoint,map)
%          failedAttempts = failedAttempts + 1;
%         continue;
%     end
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

%% distanceCost.m
function h=distanceCost(a,b)
	h = sqrt(sum((a-b).^2, 2));
   % h = norm(a-b);
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
for r=0:step:sqrt(sum((nowpos-newPos).^2))
    posCheck=nowpos+r.*[sin(dir) cos(dir)];
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

 %% interpo.m 线�?�插�?
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

%% jianzhi.m
function simplepath=jianzhi(path,map)
threshold_theta = 50;
[l,w]=size(path);
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
    theta = atan2((path(end,1)-simplepath(end,1)),(path(end,2)-simplepath(end,2))); %�?后一个点与终点之间的角度�?�?
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


%% BaseFunction.m文件
function Nik_u = BaseFunction(i, k , u, U)
% 计算基函数Ni,k(u),NodeVector为节点向�?
 
if k == 0       % 0次B样条
    if (u >= U(i+1)) && (u <U(i+2))
        Nik_u = 1.0;
    else
        Nik_u = 0.0;
    end
else
    Length1 = U(i+k+1) - U(i+1);
    Length2 = U(i+k+2) - U(i+2);      % 支撑区间的长�?
    if Length1 == 0.0       % 规定0/0 = 0
        Length1 = 1.0;
    end
    if Length2 == 0.0
        Length2 = 1.0;
    end
    Nik_u = (u - U(i+1)) / Length1 * BaseFunction(i, k-1, u, U) ...
        + (U(i+k+2) - u) / Length2 * BaseFunction(i+1, k-1, u, U);
end
end


%% SplinePoint.m  B样条的绘图函�?
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
 %%    checkNeighboor
function checkneighboor=checkNeighboor(newPoint,map)
m =6;
checkneighboor = true;
for degree = pi/4:pi/4:2*pi
    nerghboor = round(newPoint + m * [sin(degree)  cos(degree)]);
    if ~feasiblePoint(nerghboor,map)
        checkneighboor=false;
        break;
    end
end
end