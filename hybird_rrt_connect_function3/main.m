clear;clc;

%load map
I = imread('lianjie.jpg');
A1 = imbinarize(I);
global map thickenline ;
map = A1(:,:,1);

%find thicken line
Map_Astar =bwmorph(~map,'thicken',inf);
[a,b] = find(Map_Astar ==0);
thickenline = [a,b];
Map_Astar = ~Map_Astar;
%clear 
clear I A1 a b ;

% init
Start = [50 600];
Goal = [450 50];

if (iscollision(Start) == true)
    error("Start lies on an obstacle or outside map");
end

if (iscollision(Goal) == true)
    error("Goal lies on an obstacle or outside map");
end

%plot map
figure(1)
imshow(map);
axis on;
hold on;
plot(Start(2),Start(1),'g.','Markersize',25);
plot(Goal(2),Goal(1),'b.','Markersize',25);
plot(thickenline(:,2),thickenline(:,1),'.c')
hold on;

% use hybrid rrt-connect to find path
hybrid_path = hybrid_rrt_connect(Start,Goal,map,Map_Astar); 

%plot path
figure(1)
plot(hybrid_path(:,2),hybrid_path(:,1),'r','LineWidth',2);

