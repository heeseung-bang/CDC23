% load trip_data_m30_v12.mat

global CAVs prev_CAV th delta;
global u_min u_max v_min v_max;

X = sol.x;
% v = 9 % this should be the same as one in trip_generation.m

% CAVs structure
cav_fields = {'t0','v0','tf','vf','phis','path','exited','geometry'};
cav_cell = cell(length(cav_fields),1);
cav_info = cell2struct(cav_cell,cav_fields);
clear("cav_fields","cav_cell");

cav_info.exited = false;
v_min = 0.15;
v_max = 25;
u_max = 5;
u_min = -5;

delta = 6;  % safety distance [m] for rear-end safety
th = 1.5;     % Time headway [s] for lateral safety

N = length(trip);
% V0 = trip(:,4).*(1.5*ones([N,1])-1*rand([N,1]));
path = [0,1,2,3;6,0,4,5;8,9,0,7;10,11,12,0]; % path(inDir,outDir) -> 1~12

CAVs = cell(N,1);

prev_tf = zeros([4,1]); % the last CAV exit time for each direction
prev_CAV = zeros([12,1]); % the last CAV index for each path
TEST = zeros([N,1]);
TEST2 = zeros([N,1]);

%%

for n=1:N
    
    % newly entering CAV = n
    CAVs{n} = cav_info;
    CAVs{n}.t0 = trip(n,1);
    CAVs{n}.v0 = V0(n);
    CAVs{n}.path = path(trip(n,2),trip(n,3)-4);
    CAVs{n}.geometry = getGeometry(CAVs{n}.path,2*L+2*rw,rw);
    
    for i=1:(n-1)
        if CAVs{i}.t0+CAVs{i}.tf < CAVs{n}.t0
            CAVs{i}.exited = true;
        end
    end
       
    % SET Tf    
    flow_in = sum(X(:,8*(v-1)+trip(n,2)));
    flow_out = sum(X(:,8*(v-1)+trip(n,3)));
    bpr_in = t0*(1+0.15*(flow_in/gamma)^4);
    bpr_out = t0*(1+0.15*(flow_out/gamma)^4);
    tf_1 = trip(n,1) + bpr_in + bpr_out;
    tf_2 = prev_tf(trip(n,3)-4) + 1/flow_out;
    %%% currently, we are considering the exit time of the same path
    %%% sometimes, the sharing the first segment (before conflict zone) may
    %%% result in error... infeasible solution
    
    CAVs{n}.tf = max(tf_1,tf_2) - trip(n,1);
%     if prev_tf(trip(n,3)-4) == 0
%         CAVs{n}.tf = tf_1 - trip(n,1);
%     else
%         CAVs{n}.tf = tf_2 - trip(n,1);
%     end
    prev_tf(trip(n,3)-4) = CAVs{n}.tf + trip(n,1);
    
        
%%%%%%%%%%%%%%%%%%%% Coordination %%%%%%%%%%%%%%%%%%%%
    fprintf('=========================\n');
    fprintf('CAV %d\n',n);
    fprintf('Path: %d\n',CAVs{n}.path);
    fprintf('-------------------------\n');
    v_bar = L/bpr_out;
    dv = 0.1;
    done = false;
    vf = -1;
    vf1 = v_bar; vf2 = v_bar;  
    
    if n ==7
        test = 1;
    end
    
    
    while ~done
        vf1 = vf1 - dv;
        vf2 = vf2 + dv;
        vf = -1;
        if isFeasible(vf1,n)
            vf = vf1;
            done = 1;
        elseif isFeasible(vf2,n)
            vf = vf2;
            done = 1;
        elseif vf1 < v_min && vf2 > v_max
            vf = -1;
            done = 1;
        end
    end
    if(vf == -1)
        fprintf(2,'INFEASIBLE SOLUTION!!!!\n');
    else
        fprintf('t0: %.2f\n',CAVs{n}.t0);
        fprintf('tf: %.2f\n',CAVs{n}.tf+CAVs{n}.t0);
        fprintf('dt: %.2f\n',CAVs{n}.tf);
        fprintf('-------------------------\n');
        fprintf('v0: %.2f\n',CAVs{n}.v0);
        fprintf('vf: %.2f\n',vf);
        fprintf('v_bar: %.2f\n',v_bar);
        CAVs{n}.vf = vf;
        
        
        tf = CAVs{n}.tf;
        v0 = CAVs{n}.v0;
        sf = CAVs{n}.geometry.length;
        vec = ([3*tf^2,2*tf; tf^3,tf^2])\[vf-v0; sf-v0*tf];
        a = vec(1);     b = vec(2);
        CAVs{n}.phis = [a,b,v0];
    end
    prev_CAV(CAVs{n}.path) = n;
    
    
end