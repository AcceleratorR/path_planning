function Neighboors =create_around(Gridsize)

Around_Point=ones(2*Gridsize+1);
% Dummy=2*Gridsize+2;
 Mid=Gridsize+1;
%for i=1:Gridsize-1
%     Around_Point(i,i)=0;
%     Around_Point(Dummy-i,i)=0;
%     Around_Point(i,Dummy-i)=0;
%     Around_Point(Dummy-i,Dummy-i)=0;
%     Around_Point(Mid,i)=0;
%     Around_Point(Mid,Dummy-i)=0;
%     Around_Point(i,Mid)=0;
%    Around_Point(Dummy-i,Mid)=0;
% end
Around_Point(Mid,Mid)=0;

[row, col]=find(Around_Point==1);
Neighboors=[row col]-(Gridsize+1);
%check
% p = currentpoint + Neighboors;
% 
% [~,row_l] = ismember(p,thickenline,'rows');
% row_l(row_l==0)=[];
% p = thickenline(row_l,:);
% N = size(p,1);
end
