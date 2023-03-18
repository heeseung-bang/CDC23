% load sample_m30_v12.mat
%%

eps = 1e-5;

MAX_ROUTE = 7;

flow = round(sol.x,7);
route = zeros([M,MAX_ROUTE,V,2]);
route_flow = zeros([M,MAX_ROUTE]);
% For each travel demand, we assume to have MAX_ROUTE number of routes.
% Each route may pass through maximum V number of intersections and 
% last index consists of 1: intersection, 2: direction
len = zeros([M,MAX_ROUTE]);

for m=1:M   % Find route for each M
    % For each route, try to go straight as long as possible
    
    k = 0;
    while sum(flow(m,:)) > eps
        k = k+1;        % k-th route for travel demand m
        v = o(m,1);
        dir = dirNext(o(m,2));
        cur_flow = alpha(m);

        while v ~= d(m,1)
            while vNext(v,dir-4) == 0 || flow(m,8*(v-1)+dir) < eps
                dir = dir+1;
                if dir == 9
                    dir = 5;
                end
            end
            if flow(m,8*(v-1)+dir) < cur_flow
                cur_flow = flow(m,8*(v-1)+dir);
            end
            len(m,k) = len(m,k)+1;
            route(m,k,len(m,k),1) = v;
            route(m,k,len(m,k),2) = dir;
            v = vNext(v,dir-4);
        end
        
        len(m,k) = len(m,k)+1;
        route(m,k,len(m,k),1) = d(m,1);
        route(m,k,len(m,k),2) = d(m,2);
   
        v = o(m,1);
        dir = o(m,2);
        flow(m,8*(v-1)+dir) = flow(m,8*(v-1)+dir)-cur_flow;
        for i =1:len(m,k)-1
            v = route(m,k,i,1);
            dir = route(m,k,i,2);
            flow(m,8*(v-1)+dir) = flow(m,8*(v-1)+dir)-cur_flow;
            v = vNext(v,dir-4);
            dir = dirNext(dir);
            flow(m,8*(v-1)+dir) = flow(m,8*(v-1)+dir)-cur_flow;
        end
        v = d(m,1);
        dir = d(m,2);
        flow(m,8*(v-1)+dir) = flow(m,8*(v-1)+dir)-cur_flow;
        route_flow(m,k) = cur_flow;
    end
end
