% Project: "Optimal Flow-based Routing and Coordination of CAVs"
% Updated Mar.14 2023
%
% Simulation Order:
%   flow_optimization.m
%   (figure_flow_map.m)
%   route_recovery.m
%   trip_generation.m
%   coordination.m
% 
% Description: (OD:origin/destination, R:intersection)
%   flow_optimization.m - Solving flow optimization using fmincon(sqp)
%   figure_flow_map.m   - Drawing flow map (select specific travel demand)
%   route_recovery.m    - Recover all different routes connecting OD
%   trip_generation.m   - Generating CAV entry information for single R
%   coordination.m      - Solving coordination problem for single R
% 