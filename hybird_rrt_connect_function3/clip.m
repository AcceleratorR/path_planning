%% clip.m
function simplepath=clip(path)
    [len,~]=size(path);
    simplepath=zeros(len,2);

    preceding_num =1;
    last_num =2;
    N = 1;

    preceding_node = path(preceding_num,:);
    last_node=path(last_num,:);
    simplepath(N,:)=preceding_node;

    while(last_num < len)
        dir = atan2(last_node(1)-preceding_node(1),last_node(2)-preceding_node(2));
        r = 0:0.3:sqrt(sum((last_node-preceding_node).^2));
        posCheck=preceding_node+r'.*[sin(dir) cos(dir)];
        collision  = 0;
        for i = 1: length(posCheck)      
            if(iscollision(posCheck(i,:)))
                collision  = 1;
                break;
            end
        end
        if(collision == 1)
            preceding_num = last_num - 1;
            preceding_node = path(preceding_num,:);
            N = N + 1;
            simplepath(N,:)=preceding_node;
        else
           last_num = last_num + 1;
           last_node=path(last_num,:);
        end
    end
    N = N +1 ; 
    simplepath(N,:)=last_node;
    simplepath(N+1:end,:)=[];
end