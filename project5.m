%% Load data
load drone_data.mat

%% Define parameters
dt = mean(diff(times)); % estimate dt to be the average difference between time measurements
m = 0.027; % mass [kg]
g = 9.81; % gravitational constant [m/s^2]

% We are only going to use the fit data in this project
[~, ~, v, a, ~, ~, ~, ~, ~, ~, f] = p2_solution(times, pos);
r = [feval(f{1}, times), feval(f{2}, times), feval(f{3}, times)];
% Position is stored in r
% Velocity is stored in v
% Acceleration is stored in a

%% Plot 3d position
figure(1)
comet3(r(:,1), r(:,2), r(:,3))
axis image
axis vis3d
xlabel('x')
ylabel('y')
zlabel('z')

%% Calculate linear momemntum
%%%%%%%%%%%%%%%%%%%%%%%%%%%% START EDITS FOR Q1 HERE %%%%%%%%%%%%%%%%%%%%%%
% Calculate the linear momentum of the drone, L
L = m*v; % [Replace the 0 vector with your calculations]
%%%%%%%%%%%%%%%%%%%%%%%%%%%% END EDITS FOR Q1 HERE %%%%%%%%%%%%%%%%%%%%%%%%

%% Plot individual components of the momentum
coordinates = {'x', 'y', 'z'};
figure(2)
for i = 1:3
    % Plot each component of the position on its own axis
    subplot(3,1,i)
    plot(times, L(:,i), 'k')
    xlabel('Time [s]')
    ylabel(coordinates{i})
end
subplot(3,1,1)
title('Linear Momentum [kg m s^{-1}]')
drawnow

%% Calculate the total impulse relative to the start in rectangular coordinates
%%%%%%%%%%%%%%%%%%%%%%%%%%%% START EDITS FOR Q2 HERE %%%%%%%%%%%%%%%%%%%%%%
% Calculate each individual component of the linear impulse of the drone,
% I, as the difference between the initial momentum and the momentum at any
% given time.
I1x = L(1,1) - L(:,1); % [Replace the 0 with your calculations]
I1y = L(1,2) - L(:,2); % [Replace the 0 with your calculations]
I1z = L(1,3) - L(:,3); % [Replace the 0 with your calculations]

% Calculate the TOTAL force applied to the drone over time (you did this
% already in project 3)
% You can do this component-wise
Fx = m*a(:,1); % [Replace the 0 with your calculations]
Fy = m*a(:,2); % [Replace the 0 with your calculations]
Fz = m*a(:,3); % [Replace the 0 with your calculations]
% OR
% You can do this all at once, whichever you prefer.
F = [Fx, Fy, Fz]; % [Replace the 0 vector with your calculations]

% Calculate each individual component of the linear impulse of the drone,
% I, as the integral of force with respect to time. [Hint: Use cumtrapz,
% like you did in Project 4.]
I2x = cumtrapz(times, F(:,1)); % [Replace the 0 with your calculations]
I2y = cumtrapz(times, F(:,2)); % [Replace the 0 with your calculations]
I2z = cumtrapz(times, F(:,3)); % [Replace the 0 with your calculations]
%%%%%%%%%%%%%%%%%%%%%%%%%%%% END EDITS FOR Q2 HERE %%%%%%%%%%%%%%%%%%%%%%%%
I1 = [I1x, I1y, I1z];
I2 = [I2x, I2y, I2z];

%% Plot impulse
coordinates = {'x', 'y', 'z'};
figure(3)
for i = 1:3
    % Plot each component of the position on its own axis
    subplot(3,1,i)
    plot(times, I1(:,i), 'ko', times, I2(:,i), 'r')
    xlabel('Time [s]')
    ylabel(coordinates{i})
end
subplot(3,1,1)
title('Impulse')
legend('I1', 'I2', 'Location', 'Best')
drawnow

%% Calculate the angular momentum about the origin
%%%%%%%%%%%%%%%%%%%%%%%%%%%% START EDITS FOR Q3 HERE %%%%%%%%%%%%%%%%%%%%%%
% Calculate the angular momentum of the drone, H, about the origin
H = cross(r, L); % [Replace the 0 vector with your calculations]
%%%%%%%%%%%%%%%%%%%%%%%%%%%% END EDITS FOR Q3 HERE %%%%%%%%%%%%%%%%%%%%%%%%

%% Plot individual components of the angular momentum
coordinates = {'x', 'y', 'z'};
figure(4)
for i = 1:3
    % Plot each component of the position on its own axis
    subplot(3,1,i)
    plot(times, H(:,i), 'k')
    xlabel('Time [s]')
    ylabel(coordinates{i})
end
subplot(3,1,1)
title('Angular Momentum [kg m^2 s^{-1}]')
drawnow

%% Calculate the total impulse relative to the start in rectangular coordinates
%%%%%%%%%%%%%%%%%%%%%%%%%%%% START EDITS FOR Q4 HERE %%%%%%%%%%%%%%%%%%%%%%
% Calculate each individual component of the angular impulse of the drone,
% AI, as the difference between the initial angular momentum and the 
% angular momentum at any given time.
AI1x = H(1,1) - H(:,1); % [Replace the 0 with your calculations]
AI1y = H(1,2) - H(:,2); % [Replace the 0 with your calculations]
AI1z = H(1,3) - H(:,3); % [Replace the 0 with your calculations]

% Calculate the TOTAL force applied to the drone over time (you did this
% already in project 3)
% You can do this component-wise
Mx = r(:,2).*F(:,3) - r(:,3).*F(:,2); % [Replace the 0 with your calculations]
My = r(:,3).*F(:,1) - r(:,1).*F(:,3); % [Replace the 0 with your calculations]
Mz = r(:,1).*F(:,2) - r(:,2).*F(:,1); % [Replace the 0 with your calculations]
% OR
% You can do this all at once, whichever you prefer.
M = [Mx,My,Mz]; % [Replace the 0 vector with your calculations]

% Calculate each individual component of the angular impulse of the drone,
% AI, as the integral of moment with respect to time. [Hint: Use cumtrapz,
% like you did in Project 4.]
AI2x = cumtrapz(times, M(:,1)); % [Replace the 0 with your calculations]
AI2y = cumtrapz(times, M(:,2)); % [Replace the 0 with your calculations]
AI2z = cumtrapz(times, M(:,3)); % [Replace the 0 with your calculations]
%%%%%%%%%%%%%%%%%%%%%%%%%%%% END EDITS FOR Q4 HERE %%%%%%%%%%%%%%%%%%%%%%%%
AI1 = [AI1x, AI1y, AI1z];
AI2 = [AI2x, AI2y, AI2z];

%% Plot individual components of the angular impulse
coordinates = {'x', 'y', 'z'};
figure(5)
for i = 1:3
    % Plot each component of the position on its own axis
    subplot(3,1,i)
    plot(times, AI1(:,i), 'ko', times, AI2(:,i), 'r')
    xlabel('Time [s]')
    ylabel(coordinates{i})
end
subplot(3,1,1)
title('Angular Impulse [kg m^2 s^{-1}]')
legend('AI1', 'AI2', 'Location', 'Southeast')
drawnow

%%%%%%%%%% Beginning of Answer for Q5 %%%%%%%%%%
% When comparing the two momentum trends, you can see that generally they
% follow a similar path. Especially in instances of noticable acceleration
% or deceleration, there is a commonality between them. However, when we
% take a more scrutinized look at the data, you can see that there are more
% frequent and complex data variations with the angular momentum in
% comparison to the linear momentum. If we were to change the reference
% point when calculating angular momentum, I feel the change in vectors and
% cross products (which are a resultant of the position vectors) would
% result in a much different looking angular momentum graph in comparison 
% to the one we have plotted now.
%%%%%%%%%%%% Ending of Answer for Q5 %%%%%%%%%%%%


