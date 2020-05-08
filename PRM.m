%Problem 2

clc 
clear
close all

%% Control 
number_of_path_point = 200; %increase if you find trouble creating all path
range_sensor = 18; %range to detect points

%read map from pgm file
%map is 100x100 matrix (100x100 pixels)
f = imread( 'sim_map', 'pgm' );

%count number of obstacle points
num_obs = sum(sum(f==0));

%matrix for storing all obstacle positions
M = zeros(num_obs,2);
obs_cnt = 1;

%store position of obstalces into array M
for i = 1:100
    for j = 1:100
        if f(i,j) == 0
            M(obs_cnt,1) = i;
            M(obs_cnt,2) = j;
            obs_cnt = obs_cnt + 1;
        end
    end
end

%generate random uniform points in blank 100x100 matrix
m = zeros(100, 100);
num_points = number_of_path_point;
cnt = 1;

while cnt <= num_points
    point = randi(100,[1,2]); %generate random point within bounds of map
    flag_obstacle = 0; %flags if obstacle or if too close to obstacle
    for i = 1:num_obs
        if (point(1) == M(i,1) && point(2) == M(i,2)) || norm(point - M(i,:)) < 3
           flag_obstacle = 1;
        end
    end
    if flag_obstacle ~= 1 %valid position
        m(point(1), point(2)) = 1; %assign position as valid point
        cnt = cnt + 1;
    end
end

%Add start, end, and way points to path map m
m(5, 5) = 1; %start
m(95, 95) = 1; %end
% m(70, 15) = 1; %view 1
% m(90, 50) = 1; %view 2
% m(30, 95) = 1; %view 3
% m(5, 50) = 1; %view 4

%adjustment for tranposed graph
m(15, 70) = 1; %view 1 
m(50, 90) = 1; %view 2
m(95, 30) = 1; %view 3
m(50, 5) = 1; %view 4

%assign views as position variables for ease of call later on
view_1 = [15, 70];
view_2 = [50, 90];
view_3 = [95, 30];
view_4 = [50, 5];

%list of start and end for Dijkstra to loop through
trajs = [5 5 15 70;
         15 70 50 90;
         50 90 95 30;
         95 30 50 5];
         %50 5 95 95];

%count number of path points
num_path = sum(sum(m==1));

%mtrix for storing all path points
m_path = zeros(num_path,2);
path_cnt = 1;

for i = 1:100
    for j = 1:100
        if m(i,j) == 1
            m_path(path_cnt,1)=i;
            m_path(path_cnt,2)=j;
            path_cnt = path_cnt + 1;
        end
   
    end
end

%discretization size
disc = 10;

%specify radius of detection
r = range_sensor;

%list of unique path connections between two nodes
connections = zeros(10000,4);
cnt1 = 1; %count for appending connections list

%list of all path connections
all_connections = zeros(10000,4);
cnt2 = 1; %count for appending all connections list

%used nodes (current nodes do not need to be checked again)
used_nodes = zeros(num_path, 2);

for i = 1:num_path
    current_node = [m_path(i,1), m_path(i,2)];
    used_nodes(i,1) = current_node(1);
    used_nodes(i,2) = current_node(2);
    %check for nodes within range of current node
    for j=1:num_path
        next_node = [m_path(j,1), m_path(j,2)];
        
        Is_used = ismember(next_node, used_nodes, 'rows');
        
        if next_node == current_node %if nodes are the same 
            continue
        end
        
        dist = norm(current_node - next_node); %compute euclidean distance between nodes
        
        if dist < r %if node is within range
            flag = 0; %flags 1 if collision is detected, do not create path
            if next_node(1) == current_node(1) %for case where slope is infinite
                slope = 0;
                y_step = (next_node(2) - current_node(2))/disc;
            else 
                slope = (next_node(2) - current_node(2)) ./ (next_node(1) - current_node(1)); %calculate slope between two points
                y_step = 0;
            end
            x_step = (next_node(1) - current_node(1))/disc; %compute discretization step
            b = current_node(2) - slope*current_node(1);
            for k = 1:disc %discretize line between two nodes to check for obstacle collision
                check_node = [current_node(1)+x_step*k, slope*(current_node(1)+x_step*k) + b + k*y_step];           
                for l = 1:num_obs %compare the check node with obstalce nodes
                    obs_node = [M(l,1), M(l,2)];
                    collision_D = norm(check_node - obs_node); %compute distance between current point on path and obstacle
                    if collision_D < 3
                        flag = 1; %collision detection
                        break
                    end
                end
                if flag == 1
                    break
                end
            end
            
            %for storing all unique connections
            if flag~=1 && Is_used~=1 %no collisions detected and no connection
                connections(cnt, 1) = current_node(1); %current node x position
                connections(cnt, 2) = current_node(2); %current node y position
                connections(cnt, 3) = next_node(1); %next node x position
                connections(cnt, 4) = next_node(2); %next node x position

                cnt = cnt + 1;
            end
            
            %for storing all connections 
            if flag~=1 %no collisions detected 
                all_connections(cnt2, 1) = current_node(1); %current node x position
                all_connections(cnt2, 2) = current_node(2); %current node y position
                all_connections(cnt2, 3) = next_node(1); %next node x position
                all_connections(cnt2, 4) = next_node(2); %next node x position

                cnt2 = cnt2 + 1;
            end
            
        end
        
    end
end

%delete all 0 rows in connections
connections = connections(any(connections,2),:);

%delete all 0 rows in all_connections
all_connections = all_connections(any(all_connections,2),:);

%% plot map
%rerun above section if generated map is undesirable (isloltaed sections, poor connections, etc.)

figure(1)
plot(M(:,1), M(:,2), 'square')%plot obstacles
hold on

%plot paths between points
for i = 1:length(connections)
    plot([connections(i,1) connections(i,3)], [connections(i,2) connections(i,4)], 'Color', [200 200 200]/255)
end

plot(m_path(:,1), m_path(:,2), '.r') %plot PRM points

camroll(-90)

%% find path from start to view 1

%create list of path from view to view
for i = 1:length(trajs)
    Traj(i).viewpaths = [];
end

%loop for going to all views
for path_num = 1:length(trajs)
    start = trajs(path_num, 1:2); %grab corresponding start and end (goal)
    goal = trajs(path_num, 3:4);

    %set of all nodes and distances
    Dij_nodes = zeros(length(m_path),3);
    Dij_nodes(:,1) = m_path(:,1); %node x position
    Dij_nodes(:,2) = m_path(:,2); %node y position

    %list of unvisted nodes
    Dij_unvisited = m_path;

    %list of visited nodes
    Dij_visited = zeros(length(m_path),2);

    %set current node
    Dij_currentNode = start;

    %list of shortest path to points
    for i = 1:length(m_path)
        P(i).paths = [];
    end

    %set initial distances of 0 for current node and infinity for all other
    for i = 1:length(m_path)
        if Dij_nodes(i,1) == Dij_currentNode(1) && Dij_nodes(i,2) == Dij_currentNode(2)
            Dij_nodes(i,3)=0;
        else
            Dij_nodes(i,3) = 9999999999; %infinity representation
        end
    end

    %counter for while loop
    Dij_cnt = 1;

    %Dijkstra's Algorithm
    while 1 %Continues until the goal is met (break statement in loop)

        %current node index
        current_indx = find(ismember(Dij_nodes(:,[1:2]), Dij_currentNode,'rows'));

        %check if current node is already visisted
        already_visit = ismember(Dij_currentNode, Dij_visited, 'rows');

        %add current node to visited list
        Dij_visited(Dij_cnt, :) = Dij_currentNode;

        %remove visited node from unvisited list
        Dij_unvisited(find(ismember(Dij_unvisited, Dij_currentNode, 'rows')), :) = [];

        if Dij_currentNode == goal
            break
        end

        %visits all neighbors of current node
        for i = 1:length(all_connections)

            %if current node has been visited, skip and go to set new current node
            if already_visit == 1
                break
            end

            %finds the current node in list of all connections
            if all_connections(i,1)== Dij_currentNode(1) && all_connections(i,2)== Dij_currentNode(2)
                neighbor_node = [all_connections(i,3), all_connections(i,4)];%select one of the neighboring nodes
                if ismember(Dij_visited, neighbor_node, 'rows') == 1 %if the neighbor has been visited
                    continue %skip to next neighbor node
                end
                neighbor_indx = find(ismember(Dij_nodes(:,[1:2]), neighbor_node,'rows'));
                tenative_dist = norm(neighbor_node - Dij_currentNode);%calculate distance to neighbor node
                if tenative_dist + Dij_nodes(current_indx, 3) < Dij_nodes(neighbor_indx, 3)
                    Dij_nodes(neighbor_indx, 3) = tenative_dist + Dij_nodes(current_indx, 3);
                    P(neighbor_indx).paths = [P(current_indx).paths; Dij_currentNode]; %append path list for node neighbor node
                end
            end
        end

        smallest = 999999; %infinity representation

        %choose next current node (neighbor node with smallest dist)
        for j = 1:length(Dij_nodes) %go through list of nodes which contain distances
            if ismember(Dij_nodes(j,[1:2]), Dij_visited, 'rows') == 1 %if neighbor has been visited
                continue %skip
            end
            %find the neighbor with the shortest distance and set as new current node
            if Dij_nodes(j,3) < smallest
                smallest = Dij_nodes(j,3);
                Dij_currentNode = Dij_nodes(j, [1:2]);
            end
        end

        Dij_cnt = Dij_cnt + 1;
    end
    
    goal_indx = find(ismember(Dij_nodes(:,[1:2]), goal,'rows'));
    P(goal_indx).paths = [P(goal_indx).paths; goal];
    
    Traj(path_num).viewpaths = P(goal_indx).paths;
    
    path_num = path_num + 1;

end

   
%% Plot paths created by Dijkstra's

%from start to view 1
figure(2)
plot(M(:,1), M(:,2), 'square')%plot obstacles
hold on
plot(m_path(:,1), m_path(:,2), '.') %plot trajectory without error

%plot all paths 
for i = 1:length(connections)
    plot([connections(i,1) connections(i,3)], [connections(i,2) connections(i,4)], 'Color', [200 200 200]/255)
end

%plot path from start to view 1
for i = 1:length(Traj(1).viewpaths)-1
    plot([Traj(1).viewpaths(i,1) Traj(1).viewpaths(i+1,1)], [Traj(1).viewpaths(i,2) Traj(1).viewpaths(i+1,2)], 'k', 'linewidth', 2)
end
plot(Traj(1).viewpaths(:,1), Traj(1).viewpaths(:, 2), '*g') 

camroll(-90)

hold off
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%from start to view1 to view 2
figure(3)
plot(M(:,1), M(:,2), 'square')%plot obstacles
hold on
plot(m_path(:,1), m_path(:,2), '.') %plot trajectory without error

%plot all paths 
for i = 1:length(connections)
    plot([connections(i,1) connections(i,3)], [connections(i,2) connections(i,4)], 'Color', [200 200 200]/255)
end

%plot path from start to view 1
for i = 1:length(Traj(1).viewpaths)-1
    plot([Traj(1).viewpaths(i,1) Traj(1).viewpaths(i+1,1)], [Traj(1).viewpaths(i,2) Traj(1).viewpaths(i+1,2)], 'k', 'linewidth', 2)
end
plot(Traj(1).viewpaths(:,1), Traj(1).viewpaths(:, 2), '*g') 

%plot path from view 1 to view 2
for i = 1:length(Traj(2).viewpaths)-1
    plot([Traj(2).viewpaths(i,1) Traj(2).viewpaths(i+1,1)], [Traj(2).viewpaths(i,2) Traj(2).viewpaths(i+1,2)], 'k', 'linewidth', 2)
end
plot(Traj(2).viewpaths(:,1), Traj(2).viewpaths(:, 2), '*g') 

camroll(-90)

hold off

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%from start to view2 to view 3
figure(4)
plot(M(:,1), M(:,2), 'square')%plot obstacles
hold on
plot(m_path(:,1), m_path(:,2), '.') %plot trajectory without error

%plot all paths 
for i = 1:length(connections)
    plot([connections(i,1) connections(i,3)], [connections(i,2) connections(i,4)], 'Color', [200 200 200]/255)
end

%plot path from start to view 1
for i = 1:length(Traj(1).viewpaths)-1
    plot([Traj(1).viewpaths(i,1) Traj(1).viewpaths(i+1,1)], [Traj(1).viewpaths(i,2) Traj(1).viewpaths(i+1,2)], 'k', 'linewidth', 2)
end
plot(Traj(1).viewpaths(:,1), Traj(1).viewpaths(:, 2), '*g') 

%plot path from view 1 to view 2
for i = 1:length(Traj(2).viewpaths)-1
    plot([Traj(2).viewpaths(i,1) Traj(2).viewpaths(i+1,1)], [Traj(2).viewpaths(i,2) Traj(2).viewpaths(i+1,2)], 'k', 'linewidth', 2)
end
plot(Traj(2).viewpaths(:,1), Traj(2).viewpaths(:, 2), '*g') 

%plot path from view 2 to view 3
for i = 1:length(Traj(3).viewpaths)-1
    plot([Traj(3).viewpaths(i,1) Traj(3).viewpaths(i+1,1)], [Traj(3).viewpaths(i,2) Traj(3).viewpaths(i+1,2)], 'k', 'linewidth', 2)
end
plot(Traj(3).viewpaths(:,1), Traj(3).viewpaths(:, 2), '*g') 

camroll(-90)

hold off



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%from start to view 3 to view 4
figure(5)
plot(M(:,1), M(:,2), 'square')%plot obstacles
hold on
plot(m_path(:,1), m_path(:,2), '.') %plot trajectory without error

%plot all paths 
for i = 1:length(connections)
    plot([connections(i,1) connections(i,3)], [connections(i,2) connections(i,4)], 'Color', [200 200 200]/255)
end

%plot path from start to view 1
for i = 1:length(Traj(1).viewpaths)-1
    plot([Traj(1).viewpaths(i,1) Traj(1).viewpaths(i+1,1)], [Traj(1).viewpaths(i,2) Traj(1).viewpaths(i+1,2)], 'k', 'linewidth', 2)
end
plot(Traj(1).viewpaths(:,1), Traj(1).viewpaths(:, 2), '*g') 

%plot path from view 1 to view 2
for i = 1:length(Traj(2).viewpaths)-1
    plot([Traj(2).viewpaths(i,1) Traj(2).viewpaths(i+1,1)], [Traj(2).viewpaths(i,2) Traj(2).viewpaths(i+1,2)], 'k', 'linewidth', 2)
end
plot(Traj(2).viewpaths(:,1), Traj(2).viewpaths(:, 2), '*g') 

%plot path from view 2 to view 3
for i = 1:length(Traj(3).viewpaths)-1
    plot([Traj(3).viewpaths(i,1) Traj(3).viewpaths(i+1,1)], [Traj(3).viewpaths(i,2) Traj(3).viewpaths(i+1,2)], 'k', 'linewidth', 2)
end
plot(Traj(3).viewpaths(:,1), Traj(3).viewpaths(:, 2), '*g') 

%plot path from view 3 to view 4
for i = 1:length(Traj(4).viewpaths)-1
    plot([Traj(4).viewpaths(i,1) Traj(4).viewpaths(i+1,1)], [Traj(4).viewpaths(i,2) Traj(4).viewpaths(i+1,2)], 'k', 'linewidth', 2)
end
plot(Traj(4).viewpaths(:,1), Traj(4).viewpaths(:, 2), '*g') 

camroll(-90)
hold off


%% Motion model movement

syms x y delta theta time m_x m_y w

%time step of 0.1 sec
T = 0.1;

v = 0.3; %robot velocity

K_w = 0.5; %angular velocity gain

%motion model
g = [
x + T*v*cos(theta);     
y + T*v*sin(theta);
theta + w*T];

%robot parameters
b = 0.45;

%initialize starting position of robot
x_o = 5;
y_o = 5;
Theta_o = deg2rad(0);

%set position of robot to starting position
x_t = x_o;
y_t = y_o;
T_t = Theta_o;

%position/ pose of robot
mu_t = [x_t, y_t, T_t];

%matrix for storing pose of robot at every time step
pose = zeros(10000, 3);

%count for pose
pose_cnt = 1;

%index break for visiting next view
indx_break = zeros(4,1);

%loop for visiting each view
for i = 1:length(Traj)
    
    %loop for visting each waypoint on way to view
    for j = 1:length(Traj(i).viewpaths)-1
        
        %find angle of robot needed to reach next waypoint
        target_ang = atan2(Traj(i).viewpaths(j+1, 2) - mu_t(2), Traj(i).viewpaths(j+1, 1) - mu_t(1));
        
        %turn robot on spot until the angle needed to reach waypoint is hit
        while abs(mu_t(3) - target_ang) > 0.000001
            
            ang_diff = target_ang - mu_t(3);
            
            %calculate angular velocity based on difference between current and target angle
            w = (K_w/T)*ang_diff;
            
            mu_t(3) = mu_t(3) + w*T;
            
            %for the sake of plotting later, pose will only be updated after target angle has been reached
            
        end
        
        %move robot towards next way point, goes until robot is within 0.05 distance of waypoint
        %if code runs too long, it means over shot, increase distance
        %Ive been able to get within 0.05 pretty consistently 
        while norm(Traj(i).viewpaths(j+1,:) - mu_t(1:2)) > 0.05 
            mu_t(1) = mu_t(1) + T*v*cos(mu_t(3));
            mu_t(2) = mu_t(2) + T*v*sin(mu_t(3));
            
            %add current position to pose
            pose(pose_cnt, :) = mu_t;
            
            pose_cnt = pose_cnt + 1;
            
        end
        
    end
   
    indx_break(i) = pose_cnt;
    
end
%delete all 0 rows in pose
pose = pose(any(pose,2),:);


%%
%create line to show heading 
Line_L = 0.75; %length of arrow
num_lines = 50;
disc_step = round(length(pose)/num_lines) - 1;
line_start = zeros(num_lines, 2);
line_size = zeros(num_lines, 2);

for i = 1:num_lines
    line_start(i,:) = pose(i*disc_step, 1:2);
    line_size(i,1) = Line_L*cos(pose(i*disc_step, 3));
    line_size(i,2) = Line_L*sin(pose(i*disc_step, 3));
    i = i + 1;
end


%% Plot Trajectory of robot

%plot complete path of robot
figure(6)
plot(M(:,1), M(:,2), 'square')%plot obstacles
hold on
plot(m_path(:,1), m_path(:,2), '.') %plot trajectory without error

%plot all paths from PRM 
for i = 1:length(connections)
    plot([connections(i,1) connections(i,3)], [connections(i,2) connections(i,4)], 'Color', [200 200 200]/255)
end

%plot complete robot journey (all views)
plot(pose(:,1), pose(:,2), 'k', 'linewidth', 2)

%plot heading direction line
for i = 1:num_lines
    q = quiver(line_start(i,1), line_start(i,2), line_size(i,1), line_size(i,2), 2, 'r', 'linewidth', 1.5);
    q.MaxHeadSize = 20;
    i = i + 1;
end

camroll(-90)
hold off


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%plot path of robot from start to view 1
figure(7)
plot(M(:,1), M(:,2), 'square')%plot obstacles
hold on
plot(m_path(:,1), m_path(:,2), '.') %plot trajectory without error

%plot all paths from PRM 
for i = 1:length(connections)
    plot([connections(i,1) connections(i,3)], [connections(i,2) connections(i,4)], 'Color', [200 200 200]/255)
end

%plot complete robot journey (all views)
plot(pose(1:indx_break(1),1), pose(1:indx_break(1),2), 'k', 'linewidth', 2)

camroll(-90)
hold off

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%plot path of robot from start to view 1 to view 2
figure(8)
plot(M(:,1), M(:,2), 'square')%plot obstacles
hold on
plot(m_path(:,1), m_path(:,2), '.') %plot trajectory without error

%plot all paths from PRM 
for i = 1:length(connections)
    plot([connections(i,1) connections(i,3)], [connections(i,2) connections(i,4)], 'Color', [200 200 200]/255)
end

%plot robot journey
plot(pose(1:indx_break(2),1), pose(1:indx_break(2),2), 'k', 'linewidth', 2)

camroll(-90)
hold off


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%plot path of robot from start to view 1 to view 2 to view 3
figure(9)
plot(M(:,1), M(:,2), 'square')%plot obstacles
hold on
plot(m_path(:,1), m_path(:,2), '.') %plot trajectory without error

%plot all paths from PRM 
for i = 1:length(connections)
    plot([connections(i,1) connections(i,3)], [connections(i,2) connections(i,4)], 'Color', [200 200 200]/255)
end

%plot robot journey 
plot(pose(1:indx_break(3),1), pose(1:indx_break(3),2), 'k', 'linewidth', 2)

camroll(-90)
hold off

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%plot complete path of robot
figure(10)
plot(M(:,1), M(:,2), 'square')%plot obstacles
hold on
plot(m_path(:,1), m_path(:,2), '.') %plot trajectory without error

%plot all paths from PRM 
for i = 1:length(connections)
    plot([connections(i,1) connections(i,3)], [connections(i,2) connections(i,4)], 'Color', [200 200 200]/255)
end

%plot complete robot journey (all views)
plot(pose(:,1), pose(:,2), 'k', 'linewidth', 2)

camroll(-90)
hold off