clear; clc; close all;
format long g

v_min = 8.9408; % 20 mp/h
v_max = 25.29; % 70 mp/h
a_max = 5; % m/s^2
m_r_min = 10;
p_c_min = 0.01;
p_c_max =0.05;
ps = 0.35; % nulled curvature values proportion
pt = 0.75; % transition length
Nr = 40;
max_time = 600; % seconds

seed = 20000;
stream = RandStream('mt19937ar','Seed',seed);
RandStream.setGlobalStream(stream);

[arc_length, curvature, vel_knot_points,transitions] = getPlannerInput(v_min,v_max,a_max,m_r_min,p_c_min,p_c_max,ps,pt,Nr);

[x, y, psi, psi_dot] = trajectoryPlanner(arc_length,curvature, vel_knot_points);

% add transitions to x and y cumulatively

x = x + cumsum(transitions);
y = y + cumsum(transitions);
route = plotRoute(x,y);

output = [x+10,y+10];

function [arc_length, curvature, vel_knot_points, transitions] = getPlannerInput(v_min,v_max,a_max,m_r_min,p_c_min,p_c_max,ps,pt,Nr)
    % velocity
    vel_knot_points = (v_max-v_min).*rand(Nr,1) + v_min;
    rpi_min = vel_knot_points.^2./ a_max;
    % radius
    rpi = (m_r_min*rpi_min - rpi_min).*rand(length(rpi_min),1)+ rpi_min;
    [~,idxx] = sort(abs(rpi));
    rpi = rpi(idxx);
    vel_knot_points = vel_knot_points(idxx);
    % lengths
    arc_length = (p_c_max*2*pi*rpi-p_c_min*2*pi*rpi).*rand(length(rpi),1) + p_c_min*2*pi*rpi;
    transitions = arc_length*pt;
    % curvatures
    curvature = 1./rpi;
    % invert half of the curvatures
    curvature(2:2:end) = curvature(2:2:end)*-1;
    % nulling
    
    null_number = floor(ps*length(curvature));
    nulled = 0;
while nulled < null_number
    idx = randi([1 length(curvature)]);
    if (curvature(idx)==0)
       % pass
    else
        if (idx == length(curvature)) && (curvature(idx-1) ~=0 )
            curvature(idx) = 0;
            nulled = nulled + 1;
        elseif (idx == 1) && (curvature(idx) ~=0 )
            curvature(idx) = 0;
            nulled = nulled + 1;
        elseif (curvature(idx-1) ~= 0) && (curvature(idx+1) ~= 0)
            curvature(idx) = 0;
            nulled = nulled + 1;
        end
    end
end

end

function [x, y, psi, psi_dot] = trajectoryPlanner(arc_length,curvature, vel_knot_points)
    step_size = 400;
    query_arc_length = arc_length ;
    arc_length = cumsum(arc_length);
        % incremental time
    t = cumsum((arc_length(2:end)-arc_length(1:end-1))./((vel_knot_points(1:end-1)+vel_knot_points(2:end))/2))
    % longitudinal acceleration (constant for each section)
    long_acc = (vel_knot_points(2:end)-vel_knot_points(1:end-1))./t;
    t = [0;cumsum(t)];
    % create query cumulative arc lengths
%linspace(arc_length(1),arc_length(end-1),arc_length(end)/step_size)';
    % initialise output arrays
    x = zeros(length(query_arc_length),1);
    y = zeros(length(query_arc_length),1);
    psi = zeros(length(query_arc_length),1);
    psi_dot = zeros(length(query_arc_length),1);
    % calculate x, y, psi, psi_dot, for each instance
    for i = 1:length(query_arc_length)
        
        [~,idx] = min(abs(arc_length - query_arc_length(i)));
        if query_arc_length(i) < arc_length(idx)
            idx = idx - 1;
        end
        [x(i),y(i),psi(i),psi_dot(i)] = trajectoryInstance(arc_length,curvature,vel_knot_points,long_acc,t,idx,query_arc_length(i));
    end
end

function [x, y, psi, psi_dot] = trajectoryInstance(arc_length,curvature,vel_knot_points, long_acc,t,idx,query)
    % travel time - no 0 values of long_acc because no nulling performed
    tp = (-vel_knot_points(idx) + sqrt(vel_knot_points(idx)^2 + 2*long_acc(idx)*(query-arc_length(idx))))/(long_acc(idx))+ t(idx);
    % instantaenous velocity at time tp
    inst_vel = vel_knot_points(idx) + long_acc(idx)*(tp - t(idx));
    % instantaneous curvature
    k = curvature(idx) + ((curvature(idx+1)-curvature(idx))/(arc_length(idx+1)-arc_length(idx)))*query;
    % yaw rate
    yaw = inst_vel*k;
    % heading angle
    initial_heading = 0;
    yaw_angle = initial_heading + yaw*query;
    % longitudinal (x) coordinate
    x0 = 0;
    x = x0 + cos(yaw_angle)*query;
    % lateral (y) coordinate
    y0 = 0;
    y = y0 + sin(yaw_angle)*query;
    % output psi and psi_dot too
    psi = yaw_angle;
    psi_dot = yaw;
end

function [roadotuput] = plotRoute(x,y)
    route = drivingScenario('StopTime',1);
    lm = [laneMarking('Solid','Color','w'); ...
    laneMarking('Dashed','Color','y'); ...
    laneMarking('Dashed','Color','y'); ...
    laneMarking('Solid','Color','w')];
    ls = lanespec(3,'Marking',lm);
    roadcenters = [x,y];
    roadotuput = road(route,roadcenters,'Lanes',ls);
    plot(route);
end