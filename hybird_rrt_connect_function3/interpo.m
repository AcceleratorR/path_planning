%% interpo.m 
function newPath = interpo(path,num)
len = length(path);
newPath  = zeros((len-1)*num +1 ,2);
N = 1;
newPath(1,:) =path(1,:);
for i = 1:length(path)-1
    node1 = path(i,:);
    node2 = path(i+1,:);
    for t= 1/num: 1/num :1
       newnode =  [1-t,t] * [node1;node2];
       N = N +1;
       newPath(N,:) = newnode;
    end
end
newPath = round(newPath);
end

