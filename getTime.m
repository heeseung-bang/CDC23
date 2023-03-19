function time = getTime(n)

    global CAVs;
    
    eps = 1e-5;

    CPs = CAVs{n}.geometry.adjacency; % List of path with possible collision
    
    tf = CAVs{n}.tf;
    sf = CAVs{n}.geometry.length;
    tc = CAVs{n}.tc;
    time = zeros([1,length(CPs)]);
    for idx = 1:length(CPs)
        cp = CPs(idx);    % Path with possible collision

        n_idx = find(CAVs{n}.geometry.adjacency == cp);
        snc = CAVs{n}.geometry.conflictDist(n_idx);
        
        if snc <= CAVs{n}.sc
            phi = CAVs{n}.phis(1,:);
            r = roots([phi,-snc]);
            r = r(imag(r)==0);
            tnc = r(r<=tc+eps & r>0);
            time(idx) = tnc;
        else
            phi = CAVs{n}.phis(2,:);
            r = roots([phi,-(snc-CAVs{n}.sc)]);
            r = r(imag(r)==0);
            tnc = r(r<=tf-tc & r>0);
            time(idx) = tnc+tc;
        end
        
    end

end