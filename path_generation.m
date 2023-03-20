clc;

global CAVs th delta;
global u_min u_max v_min v_max;

X = sol.x;
% v = 9 % this should be the same as one in trip_generation.m

% CAVs structure
cav_fields = {'t0','v0','tf','vf','tc','sc','phis','path','exited','geometry','time'};
cav_cell = cell(length(cav_fields),1);
cav_info = cell2struct(cav_cell,cav_fields);
clear("cav_fields","cav_cell");

cav_info.exited = false;
v_min = 0.15;
v_max = 25;
u_max = 5;
u_min = -5;

delta = 5;  % safety distance [m] for rear-end safety
th = 1.5;     % Time headway [s] for lateral safety

N = length(trip);
V0 = trip(:,4); %.*(1*ones([N,1])-1*rand([N,1]));
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
    bpr_in = BPR_t0*(1+0.15*(flow_in/gamma)^4);
    bpr_out = BPR_t0*(1+0.15*(flow_out/gamma)^4);
    
    tf1 = trip(n,1) + bpr_in + bpr_out;
    tf2 = prev_tf(trip(n,3)-4) + 1/flow_out;
    CAVs{n}.tf = max(tf1,tf2) - CAVs{n}.t0;
    prev_tf(trip(n,3)-4) = CAVs{n}.tf + CAVs{n}.t0;
    CAVs{n}.vf = L/bpr_out;    
    v_avg = (CAVs{n}.v0 + CAVs{n}.vf)/2;
%     v_avg = CAVs{n}.vf;
%     CAVs{n}.vf = 22;
%     v_avg = 1;
    
    %%%%%%%%%%%%%%%%%%%%%% START COORDINATION %%%%%%%%%%%%%%%%%%%%%%
    fprintf('=========================\n');
    fprintf('CAV %d\n',n);
    fprintf('Path: %d\n',CAVs{n}.path);
    fprintf('-------------------------\n');
    
    v0 = CAVs{n}.v0;
    sf = CAVs{n}.geometry.length;
    vf = CAVs{n}.vf;
    t0 = CAVs{n}.t0;
    tf = CAVs{n}.tf;
    
        
    %%%%%%% CHECK SAFETY for single trajectory %%%%%%%
    cp = 0;
    sc = sf; vc = vf; tc = tf;
    if isFeasible(n,sc,vc,tc,0)
        
        CAVs{n}.tc = tc;
        CAVs{n}.sc = sc;
                
        vec = [3*tc^2,2*tc;tc^3,tc^2]\[vc-v0;sc-v0*tc];
        phi_n1 = [vec(1), vec(2), v0];
        phi_n2 = [0,0,0];
        
        CAVs{n}.phis = [phi_n1;phi_n2];
        CAVs{n}.time = getTime(n);
    else
        %%%%%%% CHECK SAFETY for two trajectories %%%%%%%
        CPs = CAVs{n}.geometry.adjacency;
        found = false;
        for p=1:(n-1)
            if CAVs{p}.exited
                continue;
            end
            for idx = 1:length(CPs)
                cp = CPs(idx);
                if CAVs{p}.path ~= cp
                    continue;
                end
                if n == 11
                    test = 1;
                end
                % select edge cases and check safety
                dt = CAVs{n}.t0 - CAVs{p}.t0;
                n_idx = find(CAVs{n}.geometry.adjacency == cp);
                p_idx = find(CAVs{p}.geometry.adjacency == CAVs{n}.path);
                sc = CAVs{n}.geometry.conflictDist(n_idx);
                vc = v_avg;
                
                %%%%%%% IF I want to change vc, make loop HERE!!
                
                % Check left edge case
                tc = CAVs{p}.time(p_idx)-dt-th;                
                if isFeasible(n,sc,vc,tc,p)
                    found = true;
                    
                    CAVs{n}.tc = tc;
                    CAVs{n}.sc = sc;
                    
                    vec = [3*tc^2,2*tc;tc^3,tc^2]\[vc-v0;sc-v0*tc];
                    phi_n1 = [vec(1), vec(2), v0];

                    vec = [3*(tf-tc)^2,2*(tf-tc);(tf-tc)^3,(tf-tc)^2]\[vf-vc;(sf-sc)-vc*(tf-tc)];
                    phi_n2 = [vec(1), vec(2), vc];
                    
                    CAVs{n}.phis = [phi_n1;phi_n2];
                    CAVs{n}.time = getTime(n);
                    break;
                end
                % Check right edge case
                if n == 8 && p == 7
                    test = 1;
                end
                tc = CAVs{p}.time(p_idx)-dt+th;  
                if isFeasible(n,sc,vc,tc,p)
                    found = true;
                    CAVs{n}.tc = tc;
                    CAVs{n}.sc = sc;
                    
                    vec = [3*tc^2,2*tc;tc^3,tc^2]\[vc-v0;sc-v0*tc];
                    phi_n1 = [vec(1), vec(2), v0];

                    vec = [3*(tf-tc)^2,2*(tf-tc);(tf-tc)^3,(tf-tc)^2]\[vf-vc;(sf-sc)-vc*(tf-tc)];
                    phi_n2 = [vec(1), vec(2), vc];
                    
                    CAVs{n}.phis = [phi_n1;phi_n2];
                    CAVs{n}.time = getTime(n);
                    break;
                end
            end
            if found
                break;
            end
        end
    end
        
       
    if sum(CAVs{n}.phis) == 0
        
        % FIND the preceding CAV
        sc = 0; vc = 0; tc = 0;
        for p = (n-1):-1:1
            if CAVs{p}.path == CAVs{n}.path || ~isempty(find(CAVs{n}.geometry.adjacencySplit == CAVs{p}.path,1))
%                 [sc,vc,tc] = findJunction(n,p);
                sc = CAVs{p}.sc;
                vc = v_avg;
                dt = CAVs{n}.t0 - CAVs{p}.t0;
                tc = CAVs{p}.tc-dt;
                if sc+vc+tc ~= 0
                    break;
                end
            end
        end
        if sc+vc+tc == 0
            fprintf(2','Infeasible solution!!!!!\n');
        else
            fprintf('Selected safe trajectory...\n');
            sc = sc - delta;
            CAVs{n}.tc = tc;
            CAVs{n}.sc = sc;

            vec = [3*tc^2,2*tc;tc^3,tc^2]\[vc-v0;sc-v0*tc];
            phi_n1 = [vec(1), vec(2), v0];

            vec = [3*(tf-tc)^2,2*(tf-tc);(tf-tc)^3,(tf-tc)^2]\[vf-vc;(sf-sc)-vc*(tf-tc)];
            phi_n2 = [vec(1), vec(2), vc];

            CAVs{n}.phis = [phi_n1;phi_n2];
            CAVs{n}.time = getTime(n);
        end
    else
        fprintf('cp: %d\n',cp); % if 0 -> single trajectory
    end
    
end