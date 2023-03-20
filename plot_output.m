
T = 95;

TEST_PATH = CAVs{T}.path;

figure(T);
hold on;
t0 = CAVs{T}.t0;
tf = CAVs{T}.tf;
sf = CAVs{T}.geometry.length;
v0 = CAVs{T}.v0;
vf = CAVs{T}.vf;

SIZE = t0+tf;
xlim([0,SIZE]);
ylim([0,sf]);
plot([SIZE,SIZE],[0,sf],'k-');
plot([0,SIZE],[sf,sf],'k-');

vec = ([3*tf^2,2*tf; tf^3,tf^2])\[vf-v0; sf-v0*tf];
x = linspace(t0,t0+tf);
y = zeros(size(x));
for i=1:100
    y(i) = vec(1)*(x(i)-t0)^3+vec(2)*(x(i)-t0)^2+v0*(x(i)-t0);
end
plot(x,y,'k--');

for n=1:T
    idx = find(CAVs{n}.geometry.adjacency==TEST_PATH);
    if ~isempty(idx)
        tn0 = CAVs{n}.t0;
        idx_T = find(CAVs{T}.geometry.adjacency==CAVs{n}.path);
        abs_arrival = CAVs{n}.time(idx) + CAVs{n}.t0;
        rel_arrival = abs_arrival - CAVs{T}.t0;
        dis = CAVs{T}.geometry.conflictDist(idx_T);
        plot(abs_arrival,dis,'r*');
        plot([abs_arrival-th,abs_arrival+th],[dis,dis],'r-');
        plot(abs_arrival+th,dis,'bo');
        plot(abs_arrival-th,dis,'bo');
    end
    
    if CAVs{n}.path == CAVs{T}.path
        tp0 = CAVs{n}.t0;
        tnf = CAVs{n}.tf;
        dt = CAVs{T}.t0 - CAVs{n}.t0;
        phi = CAVs{n}.phis(1,:);
        tpc = CAVs{n}.tc;
        tpf = CAVs{n}.tf;
        spc = CAVs{n}.sc;
        x = linspace(tp0,tpc+tp0);
        y = phi(1).*(x-tp0).^3+phi(2).*(x-tp0).^2+phi(3).*(x-tp0);
        plot(x,y,'k-');
        if tpc ~= tpf
            phi = CAVs{n}.phis(2,:);
            x = linspace(tpc+tp0,tpf+tp0);
            y = phi(1).*(x-tpc-tp0).^3+phi(2).*(x-tpc-tp0).^2+phi(3).*(x-tpc-tp0)+spc;
            plot(x,y,'k-');
        end
    end
    
    if any(CAVs{n}.path == CAVs{T}.geometry.adjacencySplit)
        tp0 = CAVs{n}.t0;
        tnf = CAVs{n}.tf;
        phi = CAVs{n}.phis(1,:);
        tpc = CAVs{n}.tc;
        tpf = CAVs{n}.tf;
        spc = CAVs{n}.sc;
        x = linspace(tp0,tpc+tp0);
        for i = 1:100
            y(i) = phi(1).*(x(i)-tp0).^3+phi(2).*(x(i)-tp0).^2+phi(3).*(x(i)-tp0);
            if y(i) >= CAVs{n}.geometry.split_merge_dist(1)
                break;
            end
        end
        plot(x(1:i),y(1:i),'k-');
        plot(x(i),y(i),'ko');
    end
    
    if any(CAVs{n}.path == CAVs{T}.geometry.adjacencyMerge)
        tp0 = CAVs{n}.t0;
        tnf = CAVs{n}.tf;
        phi = CAVs{n}.phis(2,:);
        if sum(phi) == 0
            phi = CAVs{n}.phis(1,:);
        end
        tpc = CAVs{n}.tc;
        tpf = CAVs{n}.tf;
        spc = CAVs{n}.sc;
        spf = CAVs{n}.geometry.length;
        if tpc == tpf
            tpc = 0;
            spc = 0;
        end
        x = linspace(tpc+tp0,tpf+tp0,400);
        first = 0;
        y = zeros(size(x));
        yy = zeros(size(x));
        for i = 1:400
            y(i) = phi(1).*(x(i)-tp0-tpc).^3+phi(2).*(x(i)-tp0-tpc).^2+phi(3).*(x(i)-tp0-tpc)+spc;
            yy(i) = CAVs{T}.geometry.length-(spf -(phi(1).*(x(i)-tp0-tpc).^3+phi(2).*(x(i)-tp0-tpc).^2+phi(3).*(x(i)-tp0-tpc)+spc));
            if y(i) >= CAVs{n}.geometry.split_merge_dist(2) && first == 0
                idx = i;
                first = 1;
            end
        end
        plot(x(idx:400),yy(idx:400),'r-');
    end
    
end