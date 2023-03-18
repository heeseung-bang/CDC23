% load trip_data_m30_v12.mat

clc;

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
th = 2;     % Time headway [s] for lateral safety

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
       
    flow_in = sum(X(:,8*(v-1)+trip(n,2)));
    flow_out = sum(X(:,8*(v-1)+trip(n,3)));
    bpr_in = t0*(1+0.15*(flow_in/gamma)^4);
    bpr_out = t0*(1+0.15*(flow_out/gamma)^4);
    
    target = bpr_in+bpr_out;
%     CAVs{n}.vf = L/bpr_out;
    v_bar = L/bpr_out;
    
    tf_1 = trip(n,1) + bpr_in + bpr_out;
    tf_2 = prev_tf(trip(n,3)-4) + 1/flow_out;
    
    target = max(tf_1,tf_2)-trip(n,1);
    
%     CAVs{n}.tf = max(tf_1,tf_2) - trip(n,1);
%     prev_tf(trip(n,3)-4) = CAVs{n}.tf + trip(n,1);
    
        
%%%%%%%%%%%%%%%%%%%% Coordination %%%%%%%%%%%%%%%%%%%%
    fprintf('=========================\n');
    fprintf('CAV %d\n',n);
    fprintf('Path: %d\n',CAVs{n}.path);
    fprintf('-------------------------\n');
    done = false;
    
    dtf = 0.05;
    tf1 = target+dtf;   tf2 = target;
    
    dv = 0.5;
    vf1 = v_bar+dv;     vf2 = v_bar;
    
    cnt = 0;
    while ~done
        tf1 = tf1 - dtf;
        tf2 = tf2 + dtf;
        tf = -1;
        vf = -1;
        v_cnt = 0;
        found = false;
        vf1 = v_bar+dv;     vf2 = v_bar;
        while ~found
            vf1 = vf1 - dv;
            vf2 = vf2 + dv;
            if isFeasible(tf1,vf1,n)
                tf = tf1;
                vf = vf1;
                done = 1;
                found = 1;
            elseif isFeasible(tf1,vf2,n)
                tf = tf1;
                vf = vf2;
                done = 1;
                found = 1;
            elseif isFeasible(tf2,vf1,n)
                tf = tf2;
                vf = vf1;
                done = 1;
                found = 1;
            elseif isFeasible(tf2,vf2,n)
                tf = tf2;
                vf = vf2;
                done = 1;
                found = 1;
            elseif v_cnt > 20
                tf = -1;
                vf = -1;
                found = true;
            else
                v_cnt = v_cnt+1;
            end
        end
        if cnt > 200 % 5s from target
            tf = -1;
            done = 1;
        else
            cnt = cnt+1;
        end
    end
    if(tf == -1)
        fprintf(2,'INFEASIBLE SOLUTION!!!!\n');
    else
        CAVs{n}.tf = tf;
        CAVs{n}.vf = vf;
        fprintf('t0: %.2f\n',CAVs{n}.t0);
        fprintf('v0: %.2f\n',CAVs{n}.v0);
        fprintf('vf: %.2f\n',vf);
        fprintf('-------------------------\n');
        fprintf('tf:\t%.2f\n',CAVs{n}.tf);
        fprintf('target:\t%.2f\n',target);
        
        v0 = CAVs{n}.v0;
%         vf = CAVs{n}.vf;
        sf = CAVs{n}.geometry.length;
        vec = ([3*tf^2,2*tf; tf^3,tf^2])\[vf-v0; sf-v0*tf];
        a = vec(1);     b = vec(2);
        CAVs{n}.phis = [a,b,v0];
    end
    prev_CAV(CAVs{n}.path) = n;
    
    
end