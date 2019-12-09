% link  path 
function finalpath = linkpath(nodelist)
    [cr,~] = size(nodelist);
    finalpath = zeros(cr,2);
    i = 1;
    while(cr>0)
        finalpath(i,:) = nodelist(cr,1:2);
        cr = nodelist(cr,3);
        i = i+1;
    end
    finalpath(i:end,:) =[];
end