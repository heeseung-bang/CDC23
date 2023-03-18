%%%%%%%%%%%%%%%%%%%%%%%% Flow Optimization %%%%%%%%%%%%%%%%%%%%%%%%

clear all; clc;

%%%%%%%%%%%% Parameters %%%%%%%%%%%%

M = 30;                     % travel demand

nRow = 3;
nCol = 4;
V = nRow*nCol;              % intersection
E = V*8;                    % number of edges (1~4:incoming,5~8:outgoing)

L = 200;                    % length from the entry to the middle of intersection [m]
c = 4.5;                    % car length [m]
v_avg = 54;                 % average speed [km/h] = 15m/s (= 33mph)
s = 10;                     % safety distance [m]
rw = 3.5;                   % road width [m]
t0 = L/(v_avg/3.6);           % free-flow travel time t_{ij}^0 [s]
gamma = 1000*v_avg/((s+c)*3600);    % capacity of the road (i,j) = (length of the road) / (car length + safety distance) [veh/s]

%%%%%%%%%%%% Graph %%%%%%%%%%%%

vNext = zeros([V,4]);               % vNext(v,dir) will show index of next V in 'dir' direction (1:E,2:N,3:W,4:S)
dirNext = [7,8,5,6,3,4,1,2];        % dirNext(e) will show index of next edge connected to the edge 'e'
for v=1:V
    if mod(v,nCol)~=0
        vNext(v,1) = v+1;
    end
    if v-nCol > 0
        vNext(v,2) = v-nCol;
    end
    if mod(v,nCol)~= 1
        vNext(v,3) = v-1;
    end
    if v+nCol <= V
        vNext(v,4) = v+nCol;
    end
end

%%%%%%%%%%%% Trip %%%%%%%%%%%%
o = zeros([M,2]);           % o(m,1) = node o(m,2) = dir (1~4)
d = zeros([M,2]);           % d(m,1) = node d(m,2) = dir (5~8)
alpha = zeros([M,1]);       % travel demand [veh/s]

vList = zeros([2*(nCol+nRow),2]);
e=0;
for i=1:nCol
    e = e+1;
    vList(e,:) = [i,2];
    e = e+1;
    vList(e,:) = [nCol*(nRow-1)+i,4];
end
for i=1:nRow
    e = e+1;
    vList(e,:) = [nCol*(i-1)+1,3];
    e = e+1;
    vList(e,:) = [nCol*i,1];
end
for m=1:M
    o(m,:) = vList(randi(2*(nCol+nRow)),:);
    
    d(m,:) = vList(randi(2*(nCol+nRow)),:);
    while abs(o(m,1)-d(m,1))<=1 || abs(o(m,1)-d(m,1)) == nCol
        d(m,:) = vList(randi(2*(nCol+nRow)),:);
    end
    d(m,2) = d(m,2)+4;
end

alpha = rand([M,1])./5;     % travel demand ~0.24 (0.2 veh/s == 1 veh per 5s max)

% o_node = randi([1,V],M,1);
% o_dir = randi([1,4],M,1);
% o = [o_node o_dir];
% 
% d_node = randi([1,V],M,1);
% d_dir = randi([5,8],M,1);
% d = [d_node d_dir];
% 

% 
% for m=1:M
%     % check if o/d are adjacent or not
%     while abs(o(m,1)-d(m,1))<=1 || abs(o(m,1)-d(m,1)) == nCol
%         d(m,1) = randi(V);
%     end
% end

%%%%%%%%%%%% Optimization Problem %%%%%%%%%%%%

prob = optimproblem;

x = optimvar('x',[M,E],'Type','continuous','LowerBound',0);     % x(m,e) == x_e^m: flow on edge e for travel demand m

sum_x = sum(x,1);                                               % sum_x(e) == x_e: total flow on edge e
BPR = t0.*(1+(0.15./(gamma.^4))*sum_x.^4);                      % BPR = t_ij^0(1+0.15*(x_ij)^4/(gamma_ij)^4)

J = sum(BPR.*sum_x);             % Cost function
prob.Objective = J;

%%%%%%%%%%%% Constraints Setup %%%%%%%%%%%%

nConst = 4*(nCol+nRow)+2*(nCol-1)*nRow+2*(nRow-1)*nCol;
cnstr1 = optimconstr(M,nConst);
cnstr2 = optimconstr(M,V);
cnstr3 = optimconstr(M,2);

for m=1:M
    idx = 1;
    for v=1:V
        sum_in = 0; sum_out = 0;
        in=0; out =0;
        
        for dir=1:4
            if o(m,1) == v && o(m,2) == dir
                in = alpha(m);
            else
                in = 0;
            end
            if d(m,1) == v && d(m,2) == dir+4
                out = alpha(m);
            else
                out = 0;
            end
                
            if vNext(v,dir) == 0
                cnstr1(m,idx) = x(m,8*(v-1)+dir) == in;
                cnstr1(m,idx+1) = x(m,8*(v-1)+dir+4) == out;
                idx = idx+2;
            else
                if o(m,1) == vNext(v,dir) && o(m,2) == dirNext(dir+4)
                    inNext = alpha(m);
                else
                    inNext = 0;
                end
                if d(m,1) == vNext(v,dir) && d(m,2) == dirNext(dir)
                    outNext = alpha(m);
                else
                    outNext = 0;
                end
                
                if dir == 1 || dir == 4
                    cnstr1(m,idx) = x(m,8*(v-1)+dir) + outNext == x(m,8*(vNext(v,dir)-1)+dirNext(dir)) + in;
                    cnstr1(m,idx+1) = x(m,8*(v-1)+dir+4) + inNext == x(m,8*(vNext(v,dir)-1)+dirNext(dir+4)) + out;
                    idx = idx+2;
                end
            end
            
            sum_in = sum_in + x(m,8*(v-1)+dir);
            sum_out = sum_out + x(m,8*(v-1)+dir+4);
        end
        
        cnstr2(m,v) = (sum_in == sum_out);
        if v == o(m,1)
            cnstr3(m,1) = (sum_in == alpha(m));
        elseif v == d(m,1)
            cnstr3(m,2) = (sum_out == alpha(m));
        end
    end
end

prob.Constraints.c1 = cnstr1;
prob.Constraints.c2 = cnstr2;
prob.Constraints.c3 = cnstr3;

find_x0

x_init.x = x0;

%%
%%%%%%%%%%%% Running Solver %%%%%%%%%%%%

opt = optimoptions(prob);
opt.MaxIterations = 1000;
opt.Algorithm = 'sqp';
% opt.OptimalityTolerance = 1e-7;
% opt.ConstraintTolerance = 1e-7;
% sol = solve(prob,'options',opt);
tic

sol = solve(prob,x_init,'options',opt);

time = toc

X = sol.x;


% save('sample_m30_v12');