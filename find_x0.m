x0 = zeros([M,E]);


for m=1:M
    x0(m,8*(o(m,1)-1)+o(m,2)) = alpha(m);
    x0(m,8*(d(m,1)-1)+d(m,2)) = alpha(m);
    
    % Implement BFS here...
    visited = zeros([V,1]);
    que = o(m,1);
    dis = 1;
    while ~isempty(que)
        v = que(1);         % pop front
        que = que(2:end);
        num = dis(1);
        dis = dis(2:end);
        
        visited(v) = num;   % mark as visited (with distance from origin)
        
        if v == d(m,1)      % if we visited destination
            break;          % done
        end
        
        for dir = 1:4
            if vNext(v,dir) ~= 0 && visited(vNext(v,dir)) == 0    % if adjacent node hasn't been explored
                que = [que vNext(v,dir)];           % put it in the que
                dis = [dis num+1];
            end
        end
    end
    
    %Backtrack to find one path
    v = d(m,1);
    route = [];
    direction = [];
    while 1
        for dir=1:4
            if vNext(v,dir) ~= 0 && visited(vNext(v,dir)) == visited(v)-1
                route = [route v];
                direction = [direction dir];
                v = vNext(v,dir);                
                break;
            end
        end
        if v == o(m,1)
            break;
        end
    end
    for i=1:length(route)
        v = route(i);
        dir = direction(i);
        if v == o(m,1)
            continue;
        else
            x0(m,8*(v-1)+dir) = alpha(m);
            x0(m,8*(vNext(v,dir)-1)+dirNext(dir)) = alpha(m);
        end
    end
end

