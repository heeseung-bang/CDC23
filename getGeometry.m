function pathdata = getGeometry(index,L,rw)
%% Paths' Numbers, the OD matrix is also made based on the following paths
%inlets:  1:East; 2:North; 3:West; 4:South
%outlets: 5:East; 6:North; 7:West; 8:South
% path #1: East to North 
% path #2: East to West
% path #3: East to South 
% 
% Path #4: North to West 
% path #5: North to South 
% path #6: North to East 
% 
% Path #7: West to South 
% path #8: West to East 
% path #9: West to North  
% 
% Path #10: South to East 
% path #11: South to North  
% path #12: South to West 

% %paths from east
% od(1,6)=1; od(1,7)=2; od(1,8)=3;
% %paths from north
% od(2,7)=4; od(2,8)=5; od(2,5)=6;
% %paths from west
% od(3,8)=7; od(3,5)=8; od(3,6)=9;
% %paths from south
% od(4,5)=10; od(4,6)=11; od(4,7)=12;
%%
th = 0.5; %timeHeadway 
%% Length

R_l = 1.5*rw; %left turn radius (For single lane road)
R_r = 0.5*rw; %right turn radius

dis_to_mz = 0.5*(L-2*rw);
right_arc = (0.5*pi*R_r);
left_arc = (0.5*pi*R_l);

S_l = (L-2*rw)+ left_arc ;%Left turning path length
S_r = (L-2*rw)+ right_arc;%Right turning path length 

S{1} = S_r;
S{2} =  L;
S{3} = S_l;

S{4} = S_r;
S{5} = L;
S{6} = S_l;

S{7} = S_r;
S{8} = L;
S{9} = S_l;

S{10}= S_r;
S{11}= L;
S{12}= S_l; 

center = L / 2; %center of the roads

%TODO FIX LATER: For now it is assmued that left turns do not have any
%conflict with each other when they are moving toward each other
%define adjacency as road segments
Adj{1} = [9,11];
Adj{2} = [11,9,6,5, 4,12];
Adj{3} = [11,12, 6,8, 7,5]; %Left turn with path 9: I might need to consider path 3->6 as well 
Adj{4} = [2,12];
Adj{5} = [7,8,9, 12, 2,3];
Adj{6} = [8,9, 10,11, 2,3];%Left turn with path 12
Adj{7} = [5,3];
Adj{8} = [10,11,12, 3, 5,6];
Adj{9} = [11,12, 1,2, 5,6]; %Left turn:
Adj{10} = [6,8];
Adj{11} = [1,2,3, 6, 8,9];
Adj{12} = [2,3, 4,5, 8,9]; %Left turn:
%conflict distance for each Adjacency matrix

Conf{1} = [dis_to_mz+right_arc, dis_to_mz+right_arc];
Conf{2} = [center-0.5*rw, center-(sqrt(2)-1)*rw, dis_to_mz+sqrt(2)*rw, center+0.5*rw, center+rw, center+rw];
Conf{3} = [asin(1/3)*R_l+dis_to_mz, asin(2/3)*R_l+dis_to_mz, acos(2/3)*R_l+dis_to_mz, acos(1/3)*R_l+dis_to_mz, left_arc+dis_to_mz, left_arc+dis_to_mz];
Conf{4} = [dis_to_mz+right_arc, dis_to_mz+right_arc];
a_prime =  (2-sqrt(2))*rw;
b_prime =  acos(2/3)*R_l;
c_prime =  acos(1/3)*R_l;
d_prime =  asin(1/3)*R_l;
e_prime =  asin(2/3)*R_l;

Conf{5} = [center+rw, center+0.5*rw, dis_to_mz+sqrt(2)*rw, dis_to_mz+a_prime, center-0.5*rw,center+rw];
Conf{6} = [dis_to_mz+left_arc,dis_to_mz+b_prime,  dis_to_mz+left_arc,dis_to_mz+c_prime, dis_to_mz+d_prime,dis_to_mz+e_prime];
Conf{7} = [dis_to_mz+right_arc, dis_to_mz+right_arc];
Conf{8} = [center+rw, center+0.5*rw, dis_to_mz+sqrt(2)*rw, dis_to_mz+a_prime, center-0.5*rw,center+rw];
Conf{9} = [dis_to_mz+left_arc,dis_to_mz+b_prime, dis_to_mz+left_arc,dis_to_mz+c_prime, dis_to_mz+d_prime,dis_to_mz+e_prime];
Conf{10} = [dis_to_mz+right_arc, dis_to_mz+right_arc];
Conf{11} = [center+rw, center+0.5*rw, dis_to_mz+sqrt(2)*rw, dis_to_mz+a_prime, center-0.5*rw,center+rw];
Conf{12} = [dis_to_mz+left_arc,dis_to_mz+b_prime, dis_to_mz+left_arc,dis_to_mz+c_prime, dis_to_mz+d_prime,dis_to_mz+e_prime];



%split_merge_dist
dist_spl_mg{1} = [dis_to_mz, dis_to_mz+right_arc]; 
dist_spl_mg{2} = [dis_to_mz, dis_to_mz+2*rw];
dist_spl_mg{3} = [dis_to_mz, dis_to_mz+left_arc];
dist_spl_mg{4} = [dis_to_mz, dis_to_mz+right_arc];
dist_spl_mg{5} = [dis_to_mz, dis_to_mz+2*rw];
dist_spl_mg{6} = [dis_to_mz, dis_to_mz+left_arc];
dist_spl_mg{7} = [dis_to_mz, dis_to_mz+right_arc];
dist_spl_mg{8} = [dis_to_mz, dis_to_mz+2*rw];
dist_spl_mg{9} = [dis_to_mz, dis_to_mz+left_arc];
dist_spl_mg{10}= [dis_to_mz, dis_to_mz+right_arc];
dist_spl_mg{11}= [dis_to_mz, dis_to_mz+2*rw];
dist_spl_mg{12}= [dis_to_mz, dis_to_mz+left_arc];
%adjacency Split
Adj_spl{1} =[2,3];
Adj_spl{2} =[1,3];
Adj_spl{3} =[1,2];
Adj_spl{4} =[5,6];
Adj_spl{5} =[4,6];
Adj_spl{6} =[4,5];
Adj_spl{7} =[8,9];
Adj_spl{8} =[7,9];
Adj_spl{9} =[7,8];
Adj_spl{10} =[11,12];
Adj_spl{11} =[10,12];
Adj_spl{12} =[10,11];
%adjaceny Merge
Adj_mg{1}=[9,11];
Adj_mg{2}=[4,12];
Adj_mg{3}=[5,7];
Adj_mg{4}=[2,12];
Adj_mg{5}=[3,7];
Adj_mg{6}=[8,10];
Adj_mg{7}=[3,5];
Adj_mg{8}=[6,10];
Adj_mg{9}=[1,11];
Adj_mg{10}=[6,8];
Adj_mg{11}=[1,9];
Adj_mg{12}=[2,4];

%generate the struct
pathdata = struct();
pathdata.length       = S{index};
pathdata.adjacency    = Adj{index};
pathdata.conflictDist = Conf{index};
pathdata.timeHeadway  = th;
pathdata.adjacencySplit = Adj_spl{index};
pathdata.adjacencyMerge = Adj_mg{index};
pathdata.split_merge_dist= dist_spl_mg{index};

end

