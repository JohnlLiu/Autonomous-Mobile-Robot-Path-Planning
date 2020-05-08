%Problem 3

clc 
clear
close all

%%

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

%generate matrix for showing RRT points 
m = zeros(100, 100);

%Add start, end, and way points to path map m
m(5, 5) = 1; %start
m(95, 95) = 1; %end
% m(45, 50) = 1; %view 1
% m(9, 50) = 1; %view 2
% m(90, 10) = 1; %view 3
% m(90, 90) = 1; %view 4

%adjustment for tranposed graph
m(50, 45) = 1; %view 1 
m(50, 9) = 1; %view 2
m(10, 90) = 1; %view 3
m(90, 90) = 1; %view 4

%assign views as position variables for ease of call later on
view_1 = [50, 45];
view_2 = [50, 9];
view_3 = [10, 90];
view_4 = [90, 90];

%list for storing reached points
trajs = [5 5 50 45;
         50 45 50 9;
         50 9 10 90;
         10 90 90 90];

%counter for number of iterations 
RRT_cnt = 1;

%increment step size (length)
RRT_step = 1; %make this a smaller number to get more points


%create structure for holding connections for each view travel
for i = 1:length(trajs)
    RRT(i).connections = [];
end

%create structure for holding holding points of each tree for each view travel
for i = 1:length(trajs)
    RRT(i).points = [];
end


for view = 1:length(trajs)
    
    %current point
    RRT_start = trajs(view, 1:2);

    %Goal point
    RRT_goal = trajs(view, 3:4);
    
    %first point is the start
    RRT(view).points(1,:) = RRT_start;
    
    cnt1 = 1; %count for appending connections list
    
    %Generate RRT
    while 1

        reached_flag = 0; %flags if generated valid point is within distance of goal

        point = randi(100,[1,2]); %generate random point within bounds of map

        %check if point already exists
        if ismember(point, RRT(view).points, 'rows')==1
            continue %skip and create new point
        end

        %find closest vertex
        min_dist = 9999999999;
        for i = 1:length(RRT(view).points(:,1))
            if RRT(view).points(i,:) ~= [0 0] %if entry is not empty (0 0)
                closest_dist = norm(point - RRT(view).points(i,:));
                if closest_dist < min_dist
                    min_dist = closest_dist;
                    RRT_current = RRT(view).points(i,:);
                end
            end
        end

        %calculate angle between current point and random point
        ang = atan2(point(2)-RRT_current(2), point(1)-RRT_current(1));

        %calculate euclidean distance between current point and random point
        dist  = norm(point - RRT_current);
        tent_dist = 0; %create tentative distance for stepping to check if past point or not
        tent_cnt = 1;%tentative count
        tent_current = RRT_current; %create tentative current point for pupose of storing connections
        col_flag = 0;

        while tent_dist < dist

            %create point along path from current point to random point
            tent_point = [RRT_current(1) + RRT_step*tent_cnt*cos(ang), RRT_current(2) + RRT_step*tent_cnt*sin(ang)];
            tent_dist = tent_dist + RRT_step;

            %check if tentative point is within specifid distance of an obstacle 
            for i = 1:num_obs
                obs_node = [M(i,1), M(i,2)];
                collision_D = norm(tent_point - obs_node); %compute distance between tentative point on path and obstacle
                if collision_D < 3 %make this a smaller number to get denser treee
                    col_flag = 1;
                    break %first point along path is too close, break and choose new random point
                end
            end
            if col_flag == 1
                break %first point along path is too close, break and choose new random point
            end

            RRT(view).points(cnt1+1,:) = tent_point;

            RRT(view).connections(cnt1, 1:2) = tent_point;
            RRT(view).connections(cnt1, 3:4) = tent_current;

            tent_current = tent_point; %update the tenative current point as previous point for connections

            cnt1 = cnt1 + 1;
            tent_cnt = tent_cnt + 1;
                                              %v make this a smaller number to get denser treee
            if norm(tent_current - RRT_goal) < 5 && col_flag == 0% once the generated point gets close enough to end
                reached_flag = 1;
                break
            end
        end

        if reached_flag == 1 %generated valid point is close enough to goal
            break
        end

    end
    
    %add goal connection
    RRT(view).connections(cnt1, 1:2) = RRT_goal;
    RRT(view).connections(cnt1, 3:4) = tent_point;
    
end

%% plot RRT map for each view path (4 in total


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%from start to view 1
figure(1)
plot(M(:,1), M(:,2), 'square')%plot obstacles
hold on

plot(trajs(1, 1), trajs(1, 2), '*') %start
plot(trajs(1, 3), trajs(1, 4), '*')%goal

for i = 1:length(RRT(1).connections)
    plot([RRT(1).connections(i,1) RRT(1).connections(i,3)], [RRT(1).connections(i,2) RRT(1).connections(i,4)], 'Color', [200 200 200]/255)
end

camroll(-90)
hold off


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%from view 1 to view 2
figure(2)
plot(M(:,1), M(:,2), 'square')%plot obstacles
hold on

plot(trajs(2, 1), trajs(2, 2), '*') %start
plot(trajs(2, 3), trajs(2, 4), '*')%goal

for i = 1:length(RRT(2).connections)
    plot([RRT(2).connections(i,1) RRT(2).connections(i,3)], [RRT(2).connections(i,2) RRT(2).connections(i,4)], 'Color', [200 200 200]/255)
end

camroll(-90)
hold off


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%from view 2 to view 3
figure(3)
plot(M(:,1), M(:,2), 'square')%plot obstacles
hold on

plot(trajs(3, 1), trajs(3, 2), '*') %start
plot(trajs(3, 3), trajs(3, 4), '*')%goal

for i = 1:length(RRT(3).connections)
    plot([RRT(3).connections(i,1) RRT(3).connections(i,3)], [RRT(3).connections(i,2) RRT(3).connections(i,4)], 'Color', [200 200 200]/255)
end

camroll(-90)
hold off


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%from view 3 to view 4
figure(4)
plot(M(:,1), M(:,2), 'square')%plot obstacles
hold on

plot(trajs(4, 1), trajs(4, 2), '*') %start
plot(trajs(4, 3), trajs(4, 4), '*')%goal

for i = 1:length(RRT(4).connections)
    plot([RRT(4).connections(i,1) RRT(4).connections(i,3)], [RRT(4).connections(i,2) RRT(4).connections(i,4)], 'Color', [200 200 200]/255)
end

camroll(-90)
hold off

%% Get shortest path

%create path list
for i = 1:length(trajs)
    RRT(i).path = [];
end

%RRT_path = zeros(10000, 2);

for viewpath = 1:length(trajs)
    

    %works backwards from goal to start
    RRT_start = trajs(viewpath, 1:2);
    RRT_goal = trajs(viewpath, 3:4);

    %first item in path is goal
    %RRT_path(1, :) = RRT_goal;

    RRT(viewpath).path(1, :) = RRT_goal;

    %first previous point from goal will be last found point from RRT
    %prev_point = RRT_points(length(RRT_points),:);

    prev_point = RRT(viewpath).points(length(RRT(viewpath).points),:);

    %counter
    path_cnt = 2; %starts at 2 since filled in first item already

    while 1

        RRT(viewpath).path(path_cnt, :) = prev_point; %append previous point to list

        if prev_point == RRT_start
            break
        end

        %finds index of current point in question (prev_point) in list ofconnections
        point_indx = find(ismember(RRT(viewpath).connections(:,[1:2]), prev_point,'rows'));

        %new previous point will be the current points connection
        prev_point = RRT(viewpath).connections(point_indx, 3:4);

        path_cnt = path_cnt + 1;

    end
end

%% plot individual paths from view to view

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%from start to view 1
figure(5)
plot(M(:,1), M(:,2), 'square')%plot obstacles
hold on

plot(trajs(1, 1), trajs(1, 2), '*') %start
plot(trajs(1, 3), trajs(1, 4), '*')%goal

for i = 1:length(RRT(1).connections)
    plot([RRT(1).connections(i,1) RRT(1).connections(i,3)], [RRT(1).connections(i,2) RRT(1).connections(i,4)], 'Color', [200 200 200]/255)
end

for j = 1:length(RRT(1).path)-1
    plot([RRT(1).path(j,1) RRT(1).path(j+1,1)], [RRT(1).path(j,2) RRT(1).path(j+1,2)], 'k', 'linewidth', 2)
end

camroll(-90)
hold off


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%from view 1 to view 2
figure(6)
plot(M(:,1), M(:,2), 'square')%plot obstacles
hold on

plot(trajs(2, 1), trajs(2, 2), '*') %start
plot(trajs(2, 3), trajs(2, 4), '*')%goal

for i = 1:length(RRT(2).connections)
    plot([RRT(2).connections(i,1) RRT(2).connections(i,3)], [RRT(2).connections(i,2) RRT(2).connections(i,4)], 'Color', [200 200 200]/255)
end

for j = 1:length(RRT(2).path)-1
    plot([RRT(2).path(j,1) RRT(2).path(j+1,1)], [RRT(2).path(j,2) RRT(2).path(j+1,2)], 'k', 'linewidth', 2)
end

camroll(-90)
hold off

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%from view 2 to view 3
figure(7)
plot(M(:,1), M(:,2), 'square')%plot obstacles
hold on

plot(trajs(3, 1), trajs(3, 2), '*') %start
plot(trajs(3, 3), trajs(3, 4), '*')%goal

for i = 1:length(RRT(3).connections)
    plot([RRT(3).connections(i,1) RRT(3).connections(i,3)], [RRT(3).connections(i,2) RRT(3).connections(i,4)], 'Color', [200 200 200]/255)
end

for j = 1:length(RRT(3).path)-1
    plot([RRT(3).path(j,1) RRT(3).path(j+1,1)], [RRT(3).path(j,2) RRT(3).path(j+1,2)], 'k', 'linewidth', 2)
end

camroll(-90)
hold off


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%from view 3 to view 4
figure(8)
plot(M(:,1), M(:,2), 'square')%plot obstacles
hold on

plot(trajs(4, 1), trajs(4, 2), '*') %start
plot(trajs(4, 3), trajs(4, 4), '*')%goal

for i = 1:length(RRT(4).connections)
    plot([RRT(4).connections(i,1) RRT(4).connections(i,3)], [RRT(4).connections(i,2) RRT(4).connections(i,4)], 'Color', [200 200 200]/255)
end

for j = 1:length(RRT(4).path)-1
    plot([RRT(4).path(j,1) RRT(4).path(j+1,1)], [RRT(4).path(j,2) RRT(4).path(j+1,2)], 'k', 'linewidth', 2)
end

camroll(-90)
hold off


%% plot cumulitive paths from view to view
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%from start to view 1
figure(9)
plot(M(:,1), M(:,2), 'square')%plot obstacles
hold on

plot(trajs(1, 1), trajs(1, 2), '*g') %start
plot(trajs(1, 3), trajs(1, 4), '*g')%goal

for i = 1:length(RRT(1).connections)
    plot([RRT(1).connections(i,1) RRT(1).connections(i,3)], [RRT(1).connections(i,2) RRT(1).connections(i,4)], 'Color', [200 200 200]/255)
end

for j = 1:length(RRT(1).path)-1
    plot([RRT(1).path(j,1) RRT(1).path(j+1,1)], [RRT(1).path(j,2) RRT(1).path(j+1,2)], 'r', 'linewidth', 2)
end

camroll(-90)
hold off

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%from start to view 1 to view 2
figure(10)
plot(M(:,1), M(:,2), 'square')%plot obstacles
hold on

plot(trajs(1, 1), trajs(1, 2), '*g') %start
plot(trajs(1, 3), trajs(1, 4), '*g') %view 1
plot(trajs(2, 3), trajs(2, 4), '*g') %view 2

%view 1 to view 2 RRT
for i = 1:length(RRT(2).connections)
    plot([RRT(2).connections(i,1) RRT(2).connections(i,3)], [RRT(2).connections(i,2) RRT(2).connections(i,4)], 'Color', [200 200 200]/255)
end

%start to view 1
for j = 1:length(RRT(1).path)-1
    plot([RRT(1).path(j,1) RRT(1).path(j+1,1)], [RRT(1).path(j,2) RRT(1).path(j+1,2)], 'k', 'linewidth', 2)
end

%view 1 to view 2
for j = 1:length(RRT(2).path)-1
    plot([RRT(2).path(j,1) RRT(2).path(j+1,1)], [RRT(2).path(j,2) RRT(2).path(j+1,2)], 'r', 'linewidth', 2)
end

camroll(-90)
hold off


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%from start to view 1 to view 2 to view 3
figure(11)
plot(M(:,1), M(:,2), 'square')%plot obstacles
hold on

plot(trajs(1, 1), trajs(1, 2), '*g') %start
plot(trajs(1, 3), trajs(1, 4), '*g') %view 1
plot(trajs(2, 3), trajs(2, 4), '*g') %view 2
plot(trajs(3, 3), trajs(3, 4), '*g') %view 3

%view 2 to view 3 RRT
for i = 1:length(RRT(3).connections)
    plot([RRT(3).connections(i,1) RRT(3).connections(i,3)], [RRT(3).connections(i,2) RRT(3).connections(i,4)], 'Color', [200 200 200]/255)
end

%start to view 1
for j = 1:length(RRT(1).path)-1
    plot([RRT(1).path(j,1) RRT(1).path(j+1,1)], [RRT(1).path(j,2) RRT(1).path(j+1,2)], 'k', 'linewidth', 2)
end

%view 1 to view 2
for j = 1:length(RRT(2).path)-1
    plot([RRT(2).path(j,1) RRT(2).path(j+1,1)], [RRT(2).path(j,2) RRT(2).path(j+1,2)], 'k', 'linewidth', 2)
end

%view 2 to view 3
for j = 1:length(RRT(3).path)-1
    plot([RRT(3).path(j,1) RRT(3).path(j+1,1)], [RRT(3).path(j,2) RRT(3).path(j+1,2)], 'r', 'linewidth', 2)
end

camroll(-90)
hold off


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%from start to view 1 to view 2 to view 3 to view 4
figure(12)
plot(M(:,1), M(:,2), 'square')%plot obstacles
hold on

plot(trajs(1, 1), trajs(1, 2), '*g') %start
plot(trajs(1, 3), trajs(1, 4), '*g') %view 1
plot(trajs(2, 3), trajs(2, 4), '*g') %view 2
plot(trajs(3, 3), trajs(3, 4), '*g') %view 3
plot(trajs(4, 3), trajs(4, 4), '*g') %view 4

%view 3 to view 4 RRT
for i = 1:length(RRT(4).connections)
    plot([RRT(4).connections(i,1) RRT(4).connections(i,3)], [RRT(4).connections(i,2) RRT(4).connections(i,4)], 'Color', [200 200 200]/255)
end

%start to view 1
for j = 1:length(RRT(1).path)-1
    plot([RRT(1).path(j,1) RRT(1).path(j+1,1)], [RRT(1).path(j,2) RRT(1).path(j+1,2)], 'k', 'linewidth', 2)
end

%view 1 to view 2
for j = 1:length(RRT(2).path)-1
    plot([RRT(2).path(j,1) RRT(2).path(j+1,1)], [RRT(2).path(j,2) RRT(2).path(j+1,2)], 'k', 'linewidth', 2)
end

%view 2 to view 3
for j = 1:length(RRT(3).path)-1
    plot([RRT(3).path(j,1) RRT(3).path(j+1,1)], [RRT(3).path(j,2) RRT(3).path(j+1,2)], 'k', 'linewidth', 2)
end

%view 3 to view 4
for j = 1:length(RRT(4).path)-1
    plot([RRT(4).path(j,1) RRT(4).path(j+1,1)], [RRT(4).path(j,2) RRT(4).path(j+1,2)], 'r', 'linewidth', 2)
end

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
pose = zeros(50000, 3);

%count for pose
pose_cnt = 1;

%index break for visiting next view
indx_break = zeros(4,1);

%loop for visiting each view
for i = 1:length(trajs)
    
    %loop for visting each waypoint on way to view
    for j = 1:length(RRT(i).path)-1
        
        %find angle of robot needed to reach next waypoint
        target_ang = atan2(RRT(i).path(length(RRT(i).path)-j, 2) - mu_t(2), RRT(i).path(length(RRT(i).path)-j, 1) - mu_t(1));
        
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
        while norm(RRT(i).path(length(RRT(i).path)-j,:) - mu_t(1:2)) > 0.05 
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


%% Plot Trajectory of Robot

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%from start to view 1 
figure(13)
plot(M(:,1), M(:,2), 'square')%plot obstacles
hold on

plot(trajs(1, 1), trajs(1, 2), '*g') %start
plot(trajs(1, 3), trajs(1, 4), '*g') %view 1

%plot robot journey from start to view 1
plot(pose(1:indx_break(1),1), pose(1:indx_break(1),2), 'r', 'linewidth', 2)

camroll(-90)
hold off


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%from start to view 1 to view 2
figure(14)
plot(M(:,1), M(:,2), 'square')%plot obstacles
hold on

plot(trajs(1, 1), trajs(1, 2), '*g') %start
plot(trajs(1, 3), trajs(1, 4), '*g') %view 1
plot(trajs(2, 3), trajs(2, 4), '*g') %view 2

plot(pose(1:indx_break(1),1), pose(1:indx_break(1),2), 'k', 'linewidth', 2)%from start to view 1
plot(pose(indx_break(1):indx_break(2),1), pose(indx_break(1):indx_break(2),2), 'r', 'linewidth', 2)%from view 1 to view 2

camroll(-90)
hold off


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%from start to view 1 to view 2 to view 3
figure(15)
plot(M(:,1), M(:,2), 'square')%plot obstacles
hold on

plot(trajs(1, 1), trajs(1, 2), '*g') %start
plot(trajs(1, 3), trajs(1, 4), '*g') %view 1
plot(trajs(2, 3), trajs(2, 4), '*g') %view 2
plot(trajs(3, 3), trajs(3, 4), '*g') %view 3

plot(pose(1:indx_break(1),1), pose(1:indx_break(1),2), 'k', 'linewidth', 2)%from start to view 1
plot(pose(indx_break(1):indx_break(2),1), pose(indx_break(1):indx_break(2),2), 'k', 'linewidth', 2)%from view 1 to view 2
plot(pose(indx_break(2):indx_break(3),1), pose(indx_break(2):indx_break(3),2), 'r', 'linewidth', 2)%from view 2 to view 3

camroll(-90)
hold off



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%from start to view 1 to view 2 to view 3 to view 4
figure(16)
plot(M(:,1), M(:,2), 'square')%plot obstacles
hold on

plot(trajs(1, 1), trajs(1, 2), '*g') %start
plot(trajs(1, 3), trajs(1, 4), '*g') %view 1
plot(trajs(2, 3), trajs(2, 4), '*g') %view 2
plot(trajs(3, 3), trajs(3, 4), '*g') %view 3
plot(trajs(4, 3), trajs(4, 4), '*g') %view 4

plot(pose(1:indx_break(1),1), pose(1:indx_break(1),2), 'k', 'linewidth', 2)%from start to view 1
plot(pose(indx_break(1):indx_break(2),1), pose(indx_break(1):indx_break(2),2), 'k', 'linewidth', 2)%from view 1 to view 2
plot(pose(indx_break(2):indx_break(3),1), pose(indx_break(2):indx_break(3),2), 'k', 'linewidth', 2)%from view 2 to view 3
plot(pose(indx_break(3):indx_break(4)-1,1), pose(indx_break(3):indx_break(4)-1,2), 'r', 'linewidth', 2)%from view 3 to view 4

camroll(-90)
hold off


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%plot heading arrows for complete path
figure(17)
plot(M(:,1), M(:,2), 'square')%plot obstacles
hold on

plot(trajs(1, 1), trajs(1, 2), '*g') %start
plot(trajs(1, 3), trajs(1, 4), '*g') %view 1
plot(trajs(2, 3), trajs(2, 4), '*g') %view 2
plot(trajs(3, 3), trajs(3, 4), '*g') %view 3
plot(trajs(4, 3), trajs(4, 4), '*g') %view 4

plot(pose(1:indx_break(4)-1,1), pose(1:indx_break(4)-1,2), 'k', 'linewidth', 2)%from start to view 1

%plot heading direction line
for i = 1:num_lines
    q = quiver(line_start(i,1), line_start(i,2), line_size(i,1), line_size(i,2), 2, 'r', 'linewidth', 1.5);
    q.MaxHeadSize = 20;
    i = i + 1;
end

camroll(-90)
hold off
