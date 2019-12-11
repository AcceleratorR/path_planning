function [row_current,dis_cn] = choose_next_node(row_current,thickenline1,dir)
%dir =1 xia    dir = -1 shang
[len,~] = size(thickenline1);
    for i = 1*dir : 1*dir : len*dir
        row_next = i + row_current;
        if(row_next < 1 || row_next >len)
            return;
        end
        if(thickenline1(row_next,3) < thickenline1(row_current,3))
            dis_cn = cal_dist(thickenline1(row_next,1:2) ,thickenline1(row_current,1:2));
            if( dis_cn< 3)
                row_current = row_next;
                break;
            else
                continue;
            end
        end
    end    
end