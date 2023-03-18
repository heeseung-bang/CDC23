% load data_m30_v12.mat
% or run (flow_optimization.m),(route_recovery.m)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% "trip" has entry time, entry direction, exit direction, flow


T = 100;    % Terminal time of the simulation [s]
v = 9;      % Specific intersection for simulation (1~V)
X = sol.x;

trip=[];
%%%%%%%%%%%%%%%%%%% IF I want to generate for all R, I can simply run for
%%%%%%%%%%%%%%%%%%% loop (for v=1:V)
for m=1:M
    K = find(len(m,:)==0,1);
    if ~K
        K = MAX_ROUTE;
    else
        K = K-1;
    end
    for k=1:K
        idx = find(route(m,k,:,1)==v);
        flow = route_flow(m,k);
        if idx
            in = o(m,2);
            if idx >1
                in = dirNext(route(m,k,idx-1,2));
            end
            in_flow = sum(X(:,8*(v-1)+in));
            v0 = L/(t0*(1+0.15*(in_flow/gamma)^4));
            out = route(m,k,idx,2);
% % % % % % This is a way to generate trip for each travel demand flow
            for t=(1/flow):(1/flow):T
                trip = [trip; t,in,out,v0];
            end
        end
    end
end
trip=sortrows(trip);
time = zeros([4,1]);
interval = zeros([4,1]);
for dir = 1:4
    interval(dir) = 1/sum(X(:,8*(v-1)+dir));
end
% % % % This is a way to generate trip for total flow for each direction
for i = 1:length(trip)
    dir = trip(i,2);
    time(dir) = time(dir)+interval(dir);
    trip(i,1) = time(dir);
end
trip=sortrows(trip);
% save('trip_data_m30_v12');