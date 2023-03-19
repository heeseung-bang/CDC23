TEST_PATH = 3;

T = 4;
figure(T);
hold on;
tf = CAVs{T}.tf;
sf = CAVs{T}.geometry.length;
v0 = CAVs{T}.v0;
vf = CAVs{T}.vf;

xlim([0,tf]);
ylim([0,sf]);
plot([tf,tf],[0,sf],'k-');
plot([0,tf],[sf,sf],'k-');

vec = ([3*tf^2,2*tf; tf^3,tf^2])\[vf-v0; sf-v0*tf];
x = linspace(0,tf);
y = zeros(size(x));
for i=1:100
    y(i) = vec(1)*x(i)^3+vec(2)*x(i)^2+v0*x(i);
end
plot(x,y,'k--');

for n=1:T-1
    idx = find(CAVs{n}.geometry.adjacency==TEST_PATH);
    if ~isempty(idx)
        idx_T = find(CAVs{T}.geometry.adjacency==CAVs{n}.path);
        abs_arrival = CAVs{n}.time(idx) + CAVs{n}.t0;
        rel_arrival = abs_arrival - CAVs{T}.t0;
        dis = CAVs{T}.geometry.conflictDist(idx_T);
        plot(rel_arrival,dis,'r*');
        plot([rel_arrival-th,rel_arrival+th],[dis,dis],'r-');
        plot(rel_arrival+th,dis,'bo');
        plot(rel_arrival-th,dis,'bo');
    end
    
    if CAVs{n}.path == CAVs{T}.path
        dt = CAVs{T}.t0 - CAVs{n}.t0;
        phi = CAVs{n}.phis(1,:);
        tpc = CAVs{n}.tc;
        tpf = CAVs{n}.tf;
        spc = CAVs{n}.sc;
        x = linspace(0,tpc-dt);
        y = phi(1).*(x+dt).^3+phi(2).*(x+dt).^2+phi(3).*(x+dt);
        plot(x,y,'k-');
        if tpc ~= tpf
            phi = CAVs{n}.phis(2,:);
            x = linspace(tpc-dt,tpf-dt);
            y = phi(1).*(x+dt-tpc).^3+phi(2).*(x+dt-tpc).^2+phi(3).*(x+dt-tpc)+spc;
            plot(x,y,'k-');
        end
    end
end