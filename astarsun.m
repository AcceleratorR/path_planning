clear;
% MAP=im2bw(imread('map1.jpg'));%read the map


% MAP=int8(zeros(128,128));
% MAP(1:64,1)=1;
% MAP(120,3:100)=1;
% MAP(125:128,40:60)=1;
% MAP(120:128,100:120)=1;
% MAP(126,100:118)=0;
% MAP(120:126,118)=0;
% MAP(100:120,100)=1;
% MAP(114:124,112:118)=0;
% MAP(1,1:128)=1;
% MAP(128,1:128)=1;
% MAP(100,1:130)=1;
% MAP(50,28:128)=1;
% 
% MAP(50,120:122)=0;
% MAP(50,30:5:128)=0;
% 
% 
% MAP(20:30,50)=1;
% MAP(1:128,1)=1;
% MAP(1:65,128)=1;
% MAP(1,1:128)=1;
% MAP(128,1:128)=1;
% MAP(10,1:50)=1;
% MAP(25,1:50)=1;
% MAP(40,40:50)=1;
% MAP(40,40:45)=1;
% MAP(80,20:40)=1;
% MAP(80:100,40)=1;
% MAP(80:100,120)=1;
% MAP(120:122,120:122)=1;
% MAP(120:122,20:25)=1;
% MAP(120:122,10:11)=1;
% MAP(125:128,10:11)=1;
% MAP(100:110,30:40)=1;
% MAP(1:20,100:128)=1;
% MAP(10:20,80:128)=1;
% MAP(20:40,80:90)=1;
% MAP(1:40,90:90)=1;
% MAP(100:105,70:80)=1;
MAP=zeros(5);
% MAP(360,340:370)=1;
% MAP(340:370,360)=1;
MAP(3,2:4)=1;
MAP(2:4,3)=1;
%start
startx=1;
starty=1;
%end
endx=5;
endy=5;

start=zeros(1,2);
final=zeros(1,2);
now=zeros(1,2);

start=[startx,starty];

now=[startx,starty];

final=[endx,endy];

%initialize
[height,width]=size(MAP);
Gscore=zeros(height,width);
Gscore_n=zeros(height,width);
Fscore=inf(height,width);
Hscore=zeros(height,width);
openlist=zeros(height,width);
closelist=zeros(height,width);
parent=zeros(height*2+width*2,2);
open=zeros(height+width,2);
nparent=1;

openlist(startx,starty)=1;

closelist=MAP;



closelist(startx,starty)=1;
 openlist(startx,starty)=1;
 while 1
     if openlist(final(1,1),final(1,2))==1;
         break;
     end
        [col,row]=find(openlist==1);
        
        %%此处应将closelist中的点去除掉,将算过的点都放到closelist中去%%%
     
        open=[col,row];
        [a,b]=size(open);
        for i =1:a;
            Gscore(open(i,1),open(i,2))=pdist2(open(i,:),start);
            Hscore(open(i,1),open(i,2))=mdis(open(i,:),final);    
            Fscore(open(i,1),open(i,2))=Gscore(open(i,1),open(i,2))+Hscore(open(i,1),open(i,2));
            openlist(open(i,1),open(i,2))=0;
            
        end
        minf=min(min(Fscore));
        [COWS,ROWS]=find(Fscore==minf);
       now(1,1)=COWS(1);
       now(1,2)=ROWS(1);    
       closelist(now(1,1),now(1,2))=1;
       
       parent(nparent,1:2)=[now(1,1),now(1,2)];
       
       listneighboors=Neighboorss(now);
        listneighboors(listneighboors(:,1)>height,:)=[];
        listneighboors(listneighboors(:,2)>width,:)=[];
       for list=1:length(listneighboors); %   neighboors
           if closelist(listneighboors(list,1),listneighboors(list,2))==1;
               continue;
           end;
           if openlist(listneighboors(list,1),listneighboors(list,2))~=1;
                   openlist(listneighboors(list,1),listneighboors(list,2))=1;
                   Gscore_n(listneighboors(list,1),listneighboors(list,2))=sqrt((listneighboors(list,1)-start(1,1))^2+(listneighboors(list,2)-start(1,2))^2);
%                    Hscore(listneighboors(list,1),listneighboors(list,2))=mdis(listneighboors(list,:),final);    
%                    Fscore(listneighboors(list,1),listneighboors(list,2))=Gscore(listneighboors(list,1),listneighboors(list,2))+Hscore(listneighboors(list,1),listneighboors(list,2));
                    if Gscore_n(listneighboors(list,1),listneighboors(list,2)) <=Gscore(now(1,1),now(1,2));
                         now(1,1:2)=[listneighboors(list,1),listneighboors(list,2)];
                         clostlist(now(1,1),now(1,2))=1;
                         nparent=nparent+1;
                    end
           end
       end
   
 end
parent=unique(parent,'rows');
% %%define distance Ma
% function y=mdis(a,b)
% y=abs(a(2)-b(2))+abs(a(1)-b(1));
% end
% 
% %%define neighboor
% function listneighboors=Neighboorss(now)
%         lup = [now(1,1)-1,now(1,2)-1];
%         up = [now(1,1)-1,now(1,2)];
%         rup = [now(1,1)-1,now(1,2)+1];
%         left = [now(1,1),now(1,2)-1];
%         right = [now(1,1),now(1,2)+1];
%         ldown = [now(1,1)+1,now(1,2)-1];
%         down = [now(1,1)+1,now(1,2)];
%         rdown = [now(1,1)+1,now(1,2)+1];
%         listneighboors=[lup;up;rup;left;right;ldown;down;rdown];
%         listneighboors(listneighboors(:,1)==0,:)=[];
%         listneighboors(listneighboors(:,2)==0,:)=[];
% 
% end










