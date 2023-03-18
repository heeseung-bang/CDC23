function feasible = isFeasible(vf,n)
    global CAVs prev_CAV th delta;
    global u_min u_max v_min v_max;
    
    %%% Check state/input constraints %%%
    
    feasible = false;
    
    if vf < v_min || vf > v_max
        return;
    end
    
    tf = CAVs{n}.tf;
    v0 = CAVs{n}.v0;
    sf = CAVs{n}.geometry.length;
    vec = ([3*tf^2,2*tf; tf^3,tf^2])\[vf-v0; sf-v0*tf];
    a = vec(1);     b = vec(2);
    u0 = 2*b;       uf = 6*a*tf;
    
    if u0 < u_min || u0 > u_max || uf < u_min || uf > u_max
        return;
    end
    
    t_star = -b/(3*a);
    v_star = 3*a*t_star^2+2*b*t_star+v0;
    
    if v_star > v_max || v_star < v_min
        return;
    end
    
    %%% Check Lateral Safety Constraints %%%
    
    CPs = CAVs{n}.geometry.adjacency; % List of path with possible collision
    
    for idx = 1:length(CPs)
        %%%%%%%%%%%%%%%%%%%%%% NEED TO CHECK ALL (NOT ONLY THE LAST ONE)
        cp = CPs(idx);    % Path with possible collision
        for p = 1:n-1
            
            if CAVs{p}.exited || CAVs{p}.path ~= cp
                continue;
            end
            
            n_idx = find(CAVs{n}.geometry.adjacency == cp);
            p_idx = find(CAVs{p}.geometry.adjacency == CAVs{n}.path);
            snc = CAVs{n}.geometry.conflictDist(n_idx);
            spc = CAVs{p}.geometry.conflictDist(p_idx);

            r = roots([a,b,v0,-snc]);
            r = r(imag(r)==0);
            tnc = r(r<tf & r>0);
            tnc_abs = tnc + CAVs{n}.t0;

            phi = CAVs{p}.phis;
            r = roots([phi,-spc]);
            r = r(imag(r)==0);
            tpc = r(r<CAVs{p}.tf & r>0);
            tpc_abs = tpc + CAVs{p}.t0;

            if abs(tpc_abs-tnc_abs) < th
%                 fprintf(2,'lateral safety violated...\n');
                return;     % Dangerous time headway between CAV n & CAV p
            end
        end
    end
    
    %%% Check Rear-End Safety Constraints %%%

    for p = 1:n-1
        if CAVs{p}.exited || CAVs{p}.path ~= CAVs{n}.path
                continue;
        end
        phi = CAVs{p}.phis;
        % check safety
        dt = CAVs{n}.t0 - CAVs{p}.t0;
        tpf = CAVs{p}.tf;
        if tpf - dt > 0
            r = roots([(3*phi(1)-3*a), (6*phi(1)*dt+2*phi(2)-2*b), (3*phi(1)*dt^2+2*phi(2)*dt+phi(3)-v0)]);
            r = r(imag(r)==0);
            te = r(r<tpf-dt & r>0);
            for i = 1:length(te) % extremum exists in shared time range
                dist_e = phi(1)*(te(i)+dt)^3+phi(2)*(te(i)+dt)^2+phi(3)*(te(i)+dt) - (a*te(i)^3+b*te(i)^2+v0*te(i));
                if dist_e < delta
                    return;
                end
            end
            dist_0 = phi(1)*(dt)^3+phi(2)*(dt)^2+phi(3)*(dt);
            if dist_0 < delta
%                 fprintf(2,'safety violation...\n'); % This is something shouldn't happen.
                return;
            end
            dist_f = phi(1)*(tpf)^3+phi(2)*(tpf)^2+phi(3)*(tpf) - (a*(tpf-dt)^3+b*(tpf-dt)^2+v0*(tpf-dt));
            if dist_f < delta
%                 fprintf(2,'rear-end safety violated...\n');
                return;
            end
        else
            %safe
        end
    end
    
    CPs = CAVs{n}.geometry.adjacencySplit; % List of path with possible collision
    
    for idx = 1:length(CPs)
        %%%%%%%%%%%%%%%%%%%%%% NEED TO CHECK ALL (NOT ONLY THE LAST ONE)
        cp = CPs(idx);    % Path with possible collision
        for p = 1:n-1
            if CAVs{p}.exited || CAVs{p}.path ~= cp
                continue;
            end
            
            

            phi = CAVs{p}.phis;
            % check safety
            dt = CAVs{n}.t0 - CAVs{p}.t0;
            tpf = CAVs{p}.tf;
            spc = CAVs{p}.geometry.split_merge_dist(1);

            r = roots([phi,-spc]);
            r = r(imag(r)==0);
            tpc = r(r<CAVs{p}.tf & r>0);
            
            if tpc - dt > 0
                r = roots([(3*phi(1)-3*a), (6*phi(1)*dt+2*phi(2)-2*b), (3*phi(1)*dt^2+2*phi(2)*dt+phi(3)-v0)]);
                r = r(imag(r)==0);
                te = r(r<tpf-dt & r>0);
                for i = 1:length(te) % extremum exists in shared time range
                    dist_e = phi(1)*(te(i)+dt)^3+phi(2)*(te(i)+dt)^2+phi(3)*(te(i)+dt) - (a*te(i)^3+b*te(i)^2+v0*te(i));
                    if dist_e < delta
                        return;
                    end
                end
                dist_0 = phi(1)*(dt)^3+phi(2)*(dt)^2+phi(3)*(dt);
                if dist_0 < delta
%                     fprintf(2,'safety violation...\n'); % This is something shouldn't happen.
                    return;
                end
                dist_f = phi(1)*(tpc)^3+phi(2)*(tpc)^2+phi(3)*(tpc) - (a*(tpc-dt)^3+b*(tpc-dt)^2+v0*(tpc-dt));
                if dist_f < delta
%                     fprintf(2,'rear-end (split) safety violated...\n')
                    return;
                end
            else
                %safe
            end
        end
    end

    CPs = CAVs{n}.geometry.adjacencyMerge; % List of path with possible collision
    
    for idx = 1:length(CPs)
        %%%%%%%%%%%%%%%%%%%%%% NEED TO CHECK ALL (NOT ONLY THE LAST ONE)
        cp = CPs(idx);    % Path with possible collision
        for p = 1:n-1
            if CAVs{p}.exited || CAVs{p}.path ~= cp
                continue;
            end

            phi = CAVs{p}.phis;
            % check safety
            dt = CAVs{n}.t0 - CAVs{p}.t0;
            tpf = CAVs{p}.tf;
            spc = CAVs{p}.geometry.split_merge_dist(2);

            r = roots([phi,-spc]);
            r = r(imag(r)==0);
            tpc = r(r<CAVs{p}.tf & r>0);
            
            if tpc - dt > 0
                r = roots([(3*phi(1)-3*a), (6*phi(1)*dt+2*phi(2)-2*b), (3*phi(1)*dt^2+2*phi(2)*dt+phi(3)-v0)]);
                r = r(imag(r)==0);
                te = r(r<tpf-dt & r>tpc-dt);
                for i = 1:length(te) % extremum exists in shared time range
                    dist_e = phi(1)*(te(i)+dt)^3+phi(2)*(te(i)+dt)^2+phi(3)*(te(i)+dt) - (a*te(i)^3+b*te(i)^2+v0*te(i));
                    if dist_e < delta
                        return;
                    end
                end
                dist_0 = phi(1)*(tpc)^3+phi(2)*(tpc)^2+phi(3)*(tpc) - (a*(tpc-dt)^3+b*(tpc-dt)^2+v0*(tpc-dt));
                if dist_0 < delta
%                     fprintf(2,'safety violation...\n'); % This is something shouldn't happen.
                    return;
                end
                dist_f = phi(1)*(tpf)^3+phi(2)*(tpf)^2+phi(3)*(tpf) - (a*(tpf-dt)^3+b*(tpf-dt)^2+v0*(tpf-dt));
                if dist_f < delta
%                     fprintf(2,'rear-end (split) safety violated...\n')
                    return;
                end
            else
                %safe
            end
        end
    end
    
    feasible = true;
    return;
end

















