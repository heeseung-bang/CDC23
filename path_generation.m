clc;

global CAVs th delta;
global u_min u_max v_min v_max;

X = sol.x;
% v = 9 % this should be the same as one in trip_generation.m

% CAVs structure
cav_fields = {'t0','v0','tf','vf','vc','tc','phis','path','exited','geometry','time'};
cav_cell = cell(length(cav_fields),1);
cav_info = cell2struct(cav_cell,cav_fields);
clear("cav_fields","cav_cell");

cav_info.exited = false;
v_min = 0.15;
v_max = 25;
u_max = 5;
u_min = -5;

delta = 6;  % safety distance [m] for rear-end safety
th = 2;     % Time headway [s] for lateral safety

N = length(trip);
% V0 = trip(:,4).*(1.5*ones([N,1])-1*rand([N,1]));
path = [0,1,2,3;6,0,4,5;8,9,0,7;10,11,12,0]; % path(inDir,outDir) -> 1~12

CAVs = cell(N,1);
prev_tf = zeros([4,1]); % saving the last exiting time at each direction (1~4)

for n = 1:N
    
    % Save newly entering CAV n
    CAVs{n} = cav_info;
    CAVs{n}.t0 = trip(n,1);
    CAVs{n}.v0 = V0(n);
    CAVs{n}.path = path(trip(n,2),trip(n,3)-4);
    CAVs{n}.geometry = getGeometry(CAVs{n}.path,2*(L+rw),rw);
    
    % Check exited CAVs
    for i = 1:(n-1)
        if CAVs{i}.t0 + CAVs{i}.tf < CAVs{n}.t0
            CAVs{i}.exited = true;
        end
    end
    
    % Estimate travel time and Fix arrival time
    flow_in = sum(X(:,8*(v-1)+trip(n,2)));
    flow_out = sum(X(:,8*(v-1)+trip(n,3)));
    bpr_in = t0*(1+0.15*(flow_in/gamma)^4);
    bpr_out = t0*(1+0.15*(flow_out/gamma)^4);
    
    tf1 = trip(n,1) + bpr_in + bpr_out;
    ft2 = prev_tf(trip(n,3)-4) + 1/flow_out;
    CAVs{n}.tf = max(tf1,tf2) - CAVs{n}.t0;
    prev_tf(trip(n,3)-4) = CAVs{n}.tf + CAVs{n}.t0;
    
    
    
    %%%%%%%%%%%%%%%%%%%%%% START COORDINATION %%%%%%%%%%%%%%%%%%%%%%
    fprintf('=========================\n');
    fprintf('CAV %d\n',n);
    fprintf('Path: %d\n',CAVs{n}.path);
    fprintf('-------------------------\n');
        
    new_coordination(n);
       
    if sum(CAVs{n}.phis) == 0
        fprintf(2','Infeasible solution!!!!!\n');
    else
        fprintf('CAV %d\n',n);
        fprintf('vc: %.2f\n',CAVs{n}.vc);
        fprintf('tc: %.2f\n',CAVs{n}.tc);
        % based on vc,tc,phis -> save time ...
        
    end
    
end