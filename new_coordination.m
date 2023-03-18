function new_coordination(n)

    global CAVs th delta;
    global u_min u_max v_min v_max;
    
    traj = [0,0,0; 0,0,0];  % [Phis1;Phis2]
    CAVs{n}.phis = traj;
        
    v0 = CAVs{n}.v0;
    vf = CAVs{n}.vf;
    v_mean = (v0+vf)/2;
    tf = CAVs{n}.tf;
    sf = CAVs{n}.geometry.length;
    
    CPs = CAVs{n}.geometry.adjacency;
    
    for idx = 1:length(CPs)     % for each CP, we find CAV with possible collision
        cp = CPs(idx);
        for p = 1:(n-1)
            if CAVs{p}.exited || CAVs{p}.path ~= cp
                continue;
            end
            n_idx = find(CAVs{n}.geometry.adjacency == cp);
            p_idx = find(CAVs{p}.geometry.adjacency == CAVs{n}.path);
            tpc = CAVs{p}.time(p_idx);
%             snc = CAVs{n}.geometry.conflictDist(n_idx); % location of the conflict point for CAV n
            tpc = CAVs{p}.time(p_idx);  % time when CAV p arriving to the conflict point
            dt = CAVs{n}.t0 - CAVs{p}.t0;
            candidate = [tpc-dt-th, tpc-dt+th];   % These are candidates for a waypoint
            for tnc_idx = 1:2
                tnc = candidate(tnc_idx);
                vc = v_mean; % maybe vc is not important!
                if new_isFeasible(vc,n,p)
                    % compute trajectory..........
                    CAVs{n}.tc = tnc;
                    CAVs{n}.vc = vc1;
                    CAVs{n}.phis = traj;
                    return;
                end
            end
            % if current choice didn't work, we can try for next 'p'
        end
    end
end