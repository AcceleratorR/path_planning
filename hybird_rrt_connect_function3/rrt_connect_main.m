clear;clc;

%load map
I = imread('xiaokou.jpg');
A1 = imbinarize(I);
global map thickenline;
map = A1(:,:,1);

%find thicken line
A2 =bwmorph(~map,'thicken',inf);
[a,b] = find(A2 ==0);
thickenline = [a,b];

%clear 
clear I A1 a b;

% init
Start = [200 200];
Goal = [5 5];

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
plot(Start(2),Start(1),'b.','Markersize',25);
plot(Goal(2),Goal(1),'g.','Markersize',25);
plot(thickenline(:,2),thickenline(:,1),'c')
% use rrt-connect to find path
hold on;
rrtconnect_path = rrt_connect(Start,Goal); 

figure(1)
plot(rrtconnect_path(:,2),rrtconnect_path(:,1),'r','LineWidth',2);

