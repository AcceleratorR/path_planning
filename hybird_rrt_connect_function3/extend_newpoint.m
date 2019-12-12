% newnode 
function newnode = extend_newpoint(rand_node,nearest_node)
global STEP;
    theta = atan2((rand_node(1)-nearest_node(1)),(rand_node(2)-nearest_node(2)));  % direction to extend sample to produce new node
    
    sline = linspace(0,STEP,30)';      % generete 10 points in line
    detect_array = round(nearest_node + sline.* [sin(theta)  cos(theta)]);
    detect_array = unique(detect_array,'rows','stable');    % generete points towards to rand_node
    
     for i = 1:length(detect_array)
        if (iscollision(detect_array(i,:)) == true)
            newnode = [];
            return;
        end    
     end 
    newnode = detect_array(end,:);

end