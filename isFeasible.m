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
    if abs(tf-tnc) > 1e-5
        vec = [3*(tf-tnc)^2,2*(tf-tnc);(tf-tnc)^3,(tf-tnc)^2]\[vf-vnc;(sf-snc)-vnc*(tf-tnc)];
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
            tmp_tnc = r(r<=tnc+1e-5 & r>0);
        else
            r = roots([phi_n2,-(sc-snc)]);
            r = r(imag(r)==0);
            tmp_tnc = r(r<=tf-tnc+1e-5 & r>0) + tnc;
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
%             if n == 8
%                 plot(t(idx),sp,'ro');
%                 plot(t(idx),sn,'b*');
%             end
            
            if sp-sn < delta
                return;
            end
        end
    end
   
    
    %%%% SPLITING PATH %%%%
    s_split = CAVs{n}.geometry.split_merge_dist(1);
    CPs = CAVs{n}.geometry.adjacencySplit; % List of path with possible collision
    for idx = 1:length(CPs)
        cp = CPs(idx);
        for p = 1:(n-1)
            if CAVs{p}.exited || CAVs{p}.path ~= cp
                    continue;
            end
                        
            tpf = CAVs{p}.tf;
            phi = CAVs{p}.phis(1,:);
            r = roots([phi,-(s_split)]);
            r = r(imag(r)==0);
            tps = r(r<=tpf & r>0); %%%%%%%%%%
            
            %%% Weird way to resolve....
            if length(tps) > 1
                tps = sort(tps);
                tps = tps(1);
            end
        
            if n == 91
                test = 1;
            end
            LIN_N = 100;
            dt = CAVs{n}.t0 - CAVs{p}.t0;
            if tps < dt
                tps = dt;
            end
            t = linspace(0,tps-dt,LIN_N);

            for i = 1:LIN_N    
                phi = CAVs{p}.phis(1,:);
                sp = phi(1)*(t(i)+dt)^3+phi(2)*(t(i)+dt)^2+phi(3)*(t(i)+dt);
                sn = phi_n1(1)*t(i)^3+phi_n1(2)*t(i)^2+phi_n1(3)*t(i);

                if sp-sn < delta
                    return;
                end
            end
        end
    end
    
    %%%% MERGING PATH %%%%
    CPs = CAVs{n}.geometry.adjacencyMerge; % List of path with possible collision
    for idx = 1:length(CPs)
        cp = CPs(idx);
        for p = 1:(n-1)
            if CAVs{p}.exited || CAVs{p}.path ~= cp
                    continue;
            end
            
            spf = CAVs{p}.geometry.length;
            tpf = CAVs{p}.tf;
            phi = CAVs{p}.phis(2,:);
            spc = CAVs{p}.sc;
            tpc = CAVs{p}.tc;
            spm = CAVs{p}.geometry.split_merge_dist(2);
            
            snm = CAVs{n}.geometry.split_merge_dist(2);
            
            if sum(phi) ~= 0 && spm >= spc
                r = roots([phi,-(spm-spc)]);
                r = r(imag(r)==0);
                tpm = r(r<=tpf-tpc & r>=0);
                tpm = tpm + tpc;
            else
                phi = CAVs{p}.phis(1,:);
                r = roots([phi,-spm]);
                r = r(imag(r)==0);
                tpm = r(r<tpf & r>=0);
            end

            LIN_N = 100;
            
            dt = CAVs{n}.t0 - CAVs{p}.t0;
            if tpm < dt
                tpm = dt;
            end
            t = linspace(tpm-dt,tpf-dt,LIN_N);
            
%             if n == 2
%                 figure(1);
%                 hold on;
%             end

            for i = 1:LIN_N    
                
                % Compute sn, sp !!!!!!!!!!!!!!!!!!!!!!!!
                
                phi = CAVs{p}.phis(2,:);
                if sum(phi) == 0
                    phi = CAVs{p}.phis(1,:);
                end
                
                tpc = CAVs{p}.tc;
                if tpc == CAVs{p}.tf
                    tpc = 0;
                end
                if sum(CAVs{p}.phis(2,:)) ~= 0
                    sp = phi(1)*(t(i)+dt-tpc)^3+phi(2)*(t(i)+dt-tpc)^2+phi(3)*(t(i)+dt-tpc)+snm;
                else
                    sp = phi(1)*(t(i)+dt-tpc)^3+phi(2)*(t(i)+dt-tpc)^2+phi(3)*(t(i)+dt-tpc)-spm+snm;
                end
                if sum(phi_n2) == 0 || t(i) < tnc
                    sn = (phi_n1(1)*t(i)^3+phi_n1(2)*t(i)^2+phi_n1(3)*t(i));
                else
                    sn = (phi_n2(1)*(t(i)-tnc)^3+phi_n2(2)*(t(i)-tnc)^2+phi_n2(3)*(t(i)-tnc)+snc);
                end
                
%                 if n == 2
%                     plot(t(i),sp,'r*');
%                     plot(t(i),sn,'b*');
%                 end

                if sp-sn < delta
                    return;
                end
            end
        end
    end
    
    feasible = true;
    return;
end


% CPs = CAVs{n}.geometry.adjacencyMerge; % List of path with possible collision
%     for idx = 1:length(CPs)
%         cp = CPs(idx);
%         for p = 1:(n-1)
%             if CAVs{p}.exited || CAVs{p}.path ~= cp
%                     continue;
%             end
%             
%             spf = CAVs{p}.geometry.length;
%             tpf = CAVs{p}.tf;
%             phi = CAVs{p}.phis(2,:);
%             spc = CAVs{p}.sc;
%             tpc = CAVs{p}.tc;
%             if sum(phi) == 0
%                 phi = CAVs{p}.phis(1,:);
%                 spc = 0;
%                 tpc = 0;
%             end
%             spm = CAVs{p}.geometry.split_merge_dist(2);
%             
%             r = roots([phi,-(spm-spc)]);
%             r = r(imag(r)==0);
%             if spm > spc
%                 tpm = r(r<=tpf-tpc & r>0);
%             else
%                 tpm = tpc;
%             end
%             
%             snm = CAVs{n}.geometry.split_merge_dist(2);
%             phi_n = phi_n2;
%             if sum(phi_n2) == 0
%                 phi_n = phi_n1;
%             end
%             r = roots([phi_n,-(snm-snc)]);
%             r = r(imag(r)==0);
%             tnm = r(r<=tf-tnc & r>0);
%             if snm == snc
%                 tnm = tnc;
%             end
%             
%             LIN_N = 100;
%             dt = CAVs{n}.t0 - CAVs{p}.t0;
%             
%             if tnm > tpf-dt
%                 feasible = true;
%                 return;
%             end
%             t = linspace(tnm,tpf-dt,LIN_N);
%             
% %             if n == 2
% %                 figure(1);
% %                 hold on;
% %             end
% 
%             for i = 1:LIN_N    
%                 phi = CAVs{p}.phis(2,:);
%                 if sum(phi) == 0
%                     phi = CAVs{p}.phis(1,:);
%                 end
%                 
%                 if t(i) > CAVs{p}.t0 + CAVs{p}.tf
%                     break;
%                 end
%                 tpc = CAVs{p}.tc;
%                 if tpc == CAVs{p}.tf
%                     tpc = 0;
%                 end
%                 sp = spf-(phi(1)*(t(i)+dt-tpc)^3+phi(2)*(t(i)+dt-tpc)^2+phi(3)*(t(i)+dt-tpc)+spc);
%                 if sum(phi_n2) == 0
%                     sn = sf-(phi_n1(1)*t(i)^3+phi_n1(2)*t(i)^2+phi_n1(3)*t(i));
%                 else
%                     sn = sf-(phi_n2(1)*(t(i)-tnc)^3+phi_n2(2)*(t(i)-tnc)^2+phi_n2(3)*(t(i)-tnc)+snc);
%                 end
% %                 if n == 2
% %                     plot(t(i),sp,'r*');
% %                     plot(t(i),sn,'b*');
% %                 end
% 
%                 if sn-sp < delta
%                     return;
%                 end
%             end
%         end
%     end
% 













