clc 
clear
close all

%read map from pgm file
%map is 100x100 matrix (100x100 pixels)
f = imread( 'sim_map', 'pgm' );

num_obs = sum(sum(f==0));

%matrix for storing all obstacle positions
M = zeros(num_obs,2);

%determine positions of all obstacle points
pos = 1;

for i=1:100
    for j=1:100
        if f(i,j) == 0
            M(pos,1) = i;
            M(pos,2) = j;
            pos = pos + 1;
        end
    end
end
%robot parameters
b = 0.45;

%initialize starting position of robot and goal
x_o = 5;
y_o = 5;
Theta_o = deg2rad(0);

%set position of robot to starting position
x_t = x_o;
y_t = y_o;
T_t = Theta_o;

%position/ pose of robot
mu_t = [x_t, y_t, T_t];

syms x y delta theta time m_x m_y v w

k_v = 0.02; %Velocity Gain
k_w = 1; %Angular Velocity Gain

%time step of 0.1 sec
T = 0.1;

%motion model
g = [
x + T*v*cos(theta);     
y + T*v*sin(theta);
theta + w*T];

%measurement model
h = [sqrt((x-m_x)^2 + (y-m_y)^2); atan2(m_y-y, m_x-x) - theta];

%record pose of robot
pose = zeros(10000,3);

%intialize counter value i
cnt = 1;

%attraction gain
k_att = 1;

%repulsion gain
k_rep = 350;

%min repulsion distance rho_o
rho_o = 5;

q_goal = [95, 95];

while (abs(mu_t(1)-q_goal(1))>0.5 || abs(mu_t(2)-q_goal(2))>0.5) && (mu_t(1) < q_goal(1) && mu_t(2) < q_goal(2))
    
    %record current position in pose
      pose(cnt,1) = mu_t(1); %x position
      pose(cnt,2) = mu_t(2); %y position
      pose(cnt,3) = mu_t(3); %theta (heading angle)
    
    %current robot position
    q = [mu_t(1), mu_t(2)];
    
    rho_goal = sqrt((q(1)-q_goal(1))^2 + (q(2)-q_goal(2))^2); 
    
    %calculate attractive forces
    U_att = (1/2)*k_att*rho_goal^2;
    F_att = -k_att*(q - q_goal);
    
    %compute Euclidean distances rho_q:
    rho_q = sqrt(sum(bsxfun(@minus, M, q).^2,2));
    
    F_rep = [0, 0];
    
    for i = 1:num_obs
        if rho_q(i) <= rho_o
            q_1 = M(i,:);
            U_rep = (1/2)*k_rep*(1/rho_q(i) - 1/rho_o)^2;
            F_rep = F_rep + k_rep*(1/rho_q(i) - 1/rho_o)*(1/rho_q(i)^2)*((q-q_1)/rho_q(i));
%          elseif rho_q(i) >= rho_o
%              U_rep = 0;
%              F_rep = 0;
        end
    end
    
    F_q = F_att + F_rep;
    
    %determine robot velocity and angular velocity scaled to resulting force
    vel = norm(F_q) * k_v;
    ang_v = -(mu_t(3) - atan2(F_q(2),F_q(1))) * k_w;
    
    mu_t = double(subs(g, [x, y, theta, v, w], [mu_t(1), mu_t(2), mu_t(3), vel, ang_v]));
    
    %update count
    cnt=cnt+1; 
end

%delete all 0 rows in pose
pose = pose(any(pose,2),:);
%%
%create line to show heading 
Line_L = 1.5; %length of arrow
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

%% plot path
figure(1)
plot(M(:,1), M(:,2), 'square')%plot obstacles
hold on
plot(pose(:,1), pose(:,2), 'k') %plot trajectory

%plot heading direction line
for i = 1:num_lines
    q = quiver(line_start(i,1), line_start(i,2), line_size(i,1), line_size(i,2), 2, 'r', 'linewidth', 1.5);
    q.MaxHeadSize = 20;
    i = i + 1;
end

camroll(-90)
