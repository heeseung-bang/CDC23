function feasible = isFeasible(n,snc,vnc,tnc,pp)
    global CAVs th delta;
    global u_min u_max v_min v_max;
    
    feasible = false;
    
    %%%%%%%%%%%%% Check State/Input Constraints %%%%%%%%%%%%%
    
    tf = CAVs{n}.tf;
    vf = CAVs{n}.vf;
    v0 = CAVs{n}.v0;
    sf = CAVs{n}.geometry.length;
    vec = ([3*tnc^2,2*tnc; tnc^3,tnc^2])\[vnc-v0; snc-v0*tnc];
    a = vec(1);     b = vec(2);
    u0 = 2*b;       uf = 6*a*tnc+2*b;
    
    if u0 < u_min || u0 > u_max || uf < u_min || uf > u_max
        return;
    end
    
    t_star = -b/(3*a);
    v_star = 3*a*t_star^2+2*b*t_star+v0;
    
    if v_star > v_max || v_star < v_min
        return;
    end
    
    vec = [3*tnc^2,2*tnc;tnc^3,tnc^2]\[vnc-v0;snc-v0*tnc];
    phi_n1 = [vec(1), vec(2), v0];
%     vec = [3*(tf-tnc)^2,2*(tf-tnc);(tf-tnc)^3,(tf-tnc)^2]\[vf-vnc;sf-vnc*(tf-tnc)];
%     phi_n2 = [vec(1), vec(2), vnc];
    if abs(tf-tnc) > 1e-5
        vec = [3*(tf-tnc)^2,2*(tf-tnc);(tf-tnc)^3,(tf-tnc)^2]\[vf-vnc;sf-vnc*(tf-tnc)];
        phi_n2 = [vec(1), vec(2), vnc];
    else
        phi_n2 = [0,0,0];
    end
    
    %%%%%%%%%%%%% Check Lateral Safety Constraints %%%%%%%%%%%%%
        
    CPs = CAVs{n}.geometry.adjacency; % List of path with possible collision
    for idx = 1:length(CPs)
        cp = CPs(idx);    % Path with possible collision
        
        n_idx = find(CAVs{n}.geometry.adjacency == cp);
        sc = CAVs{n}.geometry.conflictDist(n_idx);
        
        if sc <= snc
            r = roots([phi_n1,-sc]);
            r = r(imag(r)==0);
            tmp_tnc = r(r<tnc & r>0);
        else
            r = roots([phi_n2,-(sf-sc)]);
            r = r(imag(r)==0);
            tmp_tnc = r(r<tf-tnc & r>0) + tnc;
        end
        
        tnc_abs = tmp_tnc + CAVs{n}.t0;
        
        for p = 1:n-1    
            if CAVs{p}.exited || CAVs{p}.path ~= cp
                continue;
            end            
            if p == pp
                continue;
            end
            
            p_idx = find(CAVs{p}.geometry.adjacency == CAVs{n}.path);
            tpc = CAVs{p}.time(p_idx);
            tpc_abs = tpc + CAVs{p}.t0;

            if abs(tpc_abs-tnc_abs) < th
                return;     % Dangerous time headway between CAV n & CAV p
            end
        end
    end
    
    %%%%%%%%%%%%% Check Rear-End Safety Constraints %%%%%%%%%%%%%
    
    
    %%%% SAME PATH %%%%
    for p = 1:(n-1)
        if CAVs{p}.exited || CAVs{p}.path ~= CAVs{n}.path
            continue;    
        end
        LIN_N = 100;
        dt = CAVs{n}.t0 - CAVs{p}.t0;
        t = linspace(0,CAVs{p}.tf-dt,LIN_N);
        if n == 4
            figure(1);
            hold on;
        end
        for idx = 1:LIN_N    
            tpc = CAVs{p}.tc;
            spc = CAVs{p}.sc;
            if t(idx) <= tpc - dt
                phi = CAVs{p}.phis(1,:);
                sp = phi(1)*(t(idx)+dt)^3+phi(2)*(t(idx)+dt)^2+phi(3)*(t(idx)+dt);
            else
                phi = CAVs{p}.phis(2,:);
                sp = phi(1)*(t(idx)+dt-tpc)^3+phi(2)*(t(idx)+dt-tpc)^2+phi(3)*(t(idx)+dt-tpc)+spc;
            end
            
            if t(idx) <= tnc
                sn = phi_n1(1)*t(idx)^3+phi_n1(2)*t(idx)^2+phi_n1(3)*t(idx);
            else
                sn = phi_n2(1)*(t(idx)-tnc)^3+phi_n2(2)*(t(idx)-tnc)^2+phi_n2(3)*(t(idx)-tnc)+snc;
            end
            if n == 4
                plot(t(idx),sp,'ro');
                plot(t(idx),sn,'b*');
            end
            
            if sp-sn < delta
                return;
            end
        end
    end
    
    
    
    
    
%     %%%% SPLITING PATH %%%%
%     CPs = CAVs{n}.geometry.adjacencySplit; % List of path with possible collision
%     for idx = 1:length(CPs)
%         cp = CPs(idx);
%         for p = 1:(n-1)
%             if CAVs{p}.exited || CAVs{p}.path ~= cp
%                     continue;
%             end
%         end
%     end
%     
%     %%%% MERGING PATH %%%%
%     CPs = CAVs{n}.geometry.adjacencyMerge; % List of path with possible collision
%     for idx = 1:length(CPs)
%         cp = CPs(idx);
%         for p = 1:(n-1)
%             if CAVs{p}.exited || CAVs{p}.path ~= cp
%                     continue;
%             end
%         end
%     end
    
    feasible = true;
    return;
end

















