function [sc,vc,tc] = findJunction(n,p)

    global CAVs th delta;
    global u_min u_max v_min v_max;
    
    LIN_N = 100;
    dt = CAVs{n}.t0 - CAVs{p}.t0;
    t = linspace(0,CAVs{p}.tf-dt,LIN_N);
    tnf = CAVs{n}.tf;
    v0 = CAVs{n}.v0;
    vf = CAVs{n}.vf;
    snf = CAVs{n}.geometry.length;
    vec = [3*tnf^2,2*tnf;tnf^3,tnf^2]\[vf-v0;snf-v0*tnf];
    phi_n1 = [vec(1), vec(2), v0];

    for idx = 1:LIN_N    
        tpc = CAVs{p}.tc;
        spc = CAVs{p}.sc;
        if t(idx) <= tpc - dt
            phi = CAVs{p}.phis(1,:);
            sp = phi(1)*(t(idx)+dt)^3+phi(2)*(t(idx)+dt)^2+phi(3)*(t(idx)+dt);
            vp = 3*phi(1)*(t(idx)+dt)^2+2*phi(2)*(t(idx)+dt)+phi(3);
        else
            phi = CAVs{p}.phis(2,:);
            sp = phi(1)*(t(idx)+dt-tpc)^3+phi(2)*(t(idx)+dt-tpc)^2+phi(3)*(t(idx)+dt-tpc)+spc;
            vp = 3*phi(1)*(t(idx)+dt-tpc)^2+2*phi(2)*(t(idx)+dt-tpc)+phi(3);
        end

        sn = phi_n1(1)*t(idx)^3+phi_n1(2)*t(idx)^2+phi_n1(3)*t(idx);

        if sp <= sn
            sc = sp;
            vc = vp;
            tc = t(idx);
            return;
        end
    end
    sc = 0; vc = 0; tc = 0;
    
end

