%% Load data
load drone_data.mat
% Contains:
% times: T x 1 vector containing the time stamps of the measurements
%      e.g., times(10) gives you the time of the 10th measurement
%      There are T = 5895 measurements in the dataset
% pos: T x 3 matrix containing the 3D position of the drone at time t
%      Each position is a row in the format [x, y, z]
%      e.g., pos(10, :) gives you the 10th measured position

% Get the time step and unfiltered velocity and acceleration
dt = mean(diff(times)); % estimate dt to be the average difference between time measurements
[v_raw, a_raw] = filterN(dt, pos, 3, false); % using the standard central difference formula

%% Plot 3d position
figure(1)
comet3(pos(:,1), pos(:,2), pos(:,3))
axis image
axis vis3d
xlabel('x')
ylabel('y')
zlabel('z')

%% Fit the data
%%%%%%%%%%%%%%%%%%%%%%%%%%%% START EDITS FOR Q2 HERE %%%%%%%%%%%%%%%%%%%%%%
% Type 'doc fit' in the command window to see more information about the
% fitting function and the options available. These variables are passed to
% the fit function below
fitType = 'linearinterp';
nameValueList = {'Normalize', 'on'}; % if you do not want to use any fitOptions, then set this to be empty braces {} with nothing inside of them
% Note: You should not need to write any more lines of code, just change 
% the values in here.
%%%%%%%%%%%%%%%%%%%%%%%%%%%% END EDITS FOR Q2 HERE %%%%%%%%%%%%%%%%%%%%%%%%

% Fit the x, y, and z positions
f{1} = fit(times, pos(:,1), fitType, nameValueList{:}); % fit x data
f{2} = fit(times, pos(:,2), fitType, nameValueList{:}); % fit y data
f{3} = fit(times, pos(:,3), fitType, nameValueList{:}); % fit z data

% Calculate the position
x = feval(f{1}, times);
y = feval(f{2}, times);
z = feval(f{3}, times);
r_fit = [x y z];

% Calculate the velocity and acceleration
[vx, ax] = differentiate(f{1}, times);
[vy, ay] = differentiate(f{2}, times);
[vz, az] = differentiate(f{3}, times);
v_fit = [vx vy vz];
a_fit = [ax ay az];

%% Plot individual components of the position
coordinates = {'x', 'y', 'z'};
figure(2)
for ii = 1:3
    % Plot each component of the position on its own axis
    subplot(3,1,ii)
    h = plot(times, pos(:,ii), 'k', times, r_fit(:,ii), 'b:');
    h(1).LineWidth = 2;
    xlabel('Time [s]')
    ylabel(coordinates{ii})
end
subplot(3,1,1)
title('Position')
legend('Raw data', 'Fit', 'Location', 'Best')

% Plot velocity
figure(3)
for ii = 1:3
    % Plot each component of the velocity on its own axis
    subplot(3,1,ii)
    h = plot(times, v_raw(:,ii), 'k', times, v_fit(:,ii), 'b:');
    h(1).LineWidth = 2;
    xlabel('Time [s]')
    ylabel(['d' coordinates{ii} '/dt'])
end
subplot(3,1,1)
title('Velocity')
legend('Central difference', 'Fit', 'Location', 'Best')

% Plot acceleration
figure(4)
for ii = 1:3
    % Plot each component of the acceleration on its own axis
    subplot(3,1,ii)
    h = plot(times, a_raw(:,ii), 'k', times, a_fit(:,ii), 'b:');
    h(1).LineWidth = 2;
    xlabel('Time [s]')
    ylabel(['d^2' coordinates{ii} '/dt^2'])
    ylim([-10 10])
end
subplot(3,1,1)
title('Acceleration')
legend('Central difference', 'Fit', 'Location', 'Best')

%% Calculate numerical derivatives
%%%%%%%%%%%%%%%%%%%%%%%%%%%% START EDITS FOR Q3 HERE %%%%%%%%%%%%%%%%%%%%%%
% These variables are passed to the filterN function below.
N = 75; % can be any odd number >= 3
onesided = true; % can be either false or true
% You should not need to write any more lines of code, just change the
% values in here.
%%%%%%%%%%%%%%%%%%%%%%%%%%%% END EDITS FOR Q3 HERE %%%%%%%%%%%%%%%%%%%%%%%%

% Calculate smoothed velocity and acceleration
[v_smooth, a_smooth] = filterN(dt, pos, N, onesided); % using the parameters that you select

%% Plot data
% Plot velocity
figure(3)
for ii = 1:3
    % Plot each component of the velocity on its own axis
    subplot(3,1,ii)
    h = plot(times, v_raw(:,ii), 'k', times, v_fit(:,ii), 'b:', times, v_smooth(:,ii), 'r--');
    h(1).LineWidth = 2;
    xlabel('Time [s]')
    ylabel(['d' coordinates{ii} '/dt'])
end
subplot(3,1,1)
title('Velocity')
legend('Central difference', 'Fit', 'Smoothed', 'Location', 'Best')

% Plot acceleration
figure(4)
for ii = 1:3
    % Plot each component of the acceleration on its own axis
    subplot(3,1,ii)
    h = plot(times, a_raw(:,ii), 'k', times, a_fit(:,ii), 'b:', times, a_smooth(:,ii), 'r--');
    h(1).LineWidth = 2;
    xlabel('Time [s]')
    ylabel(['d^2' coordinates{ii} '/dt^2'])
    ylim([-10 10])
end
subplot(3,1,1)
title('Acceleration')
legend('Central difference', 'Fit', 'Smoothed', 'Location', 'Best')

%% Pick either the fit data or the smoothed data to continue
%%%%%%%%%%%%%%%%%%%%%%%%%%%% START EDITS HERE %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Comment out one of these two blocks
%v = v_fit;
%a = a_fit;

v = v_smooth;
a = a_smooth;
%%%%%%%%%%%%%%%%%%%%%%%%%%%% END EDITS HERE %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%% Calculate the forces in rectangular coordinates
%%%%%%%%%%%%%%%%%%%%%%%%%%%% START EDITS FOR Q5 HERE %%%%%%%%%%%%%%%%%%%%%%
m = 0.027; % mass in kg

% Calculate the forces using the smoothed or fit acceleration (your choice)
% Recall: the variable that stores the fit acceleration is a_fit and the 
% variable that stores the fit acceleration is a_smooth
Fx = m*a(:,1);
Fy = m*a(:,2);
Fz = m*a(:,3);
%%%%%%%%%%%%%%%%%%%%%%%%%%%% END EDITS FOR Q5 HERE %%%%%%%%%%%%%%%%%%%%%%%%

%% Plot the forces in rectangular components
F = [Fx, Fy, Fz];

% Plot forces
figure(5)
for ii = 1:3
    % Plot each component of the force on its own axis
    subplot(3,1,ii)
    plot(times, F(:,ii), 'b')
    xlabel('Time [s]')
    ylabel(['F_' coordinates{ii}])
    ylim([-0.2 0.2])
end
subplot(3,1,1)
title('Forces in Rectangular Coordinates')

%% Calculate the normal and tangential accelerations
T = size(times, 1); % number of datapoints

% Initialize the data arrays
v_mag = zeros(T, 1); % magnitude of velocity
v_unit = v; % unit vector in the direction of the velocity
for t = 1:T
    % Calculate the data
    v_mag(t) = vecnorm(v(t,:), 2);
    v_unit(t,:) = v_unit(t,:) / v_mag(t);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%% START EDITS FOR Q6 HERE %%%%%%%%%%%%%%%%%%%%%%
% You should write out the lines of code that you need to calculate the
% magnitudes of the normal and tangential accelerations in this code block.
% Store the tangential acceleration in a vector at and the normal
% acceleration in a vector an.

at = dot(a, v_unit, 2); % tangential acceleration

an = vecnorm(cross(v_unit, cross(v_unit, a)), 2, 2); % normal acceleration

%%%%%%%%%%%%%%%%%%%%%%%%%%%% END EDITS FOR Q6 HERE %%%%%%%%%%%%%%%%%%%%%%%%

%% Calculate tangential and normal forces
%%%%%%%%%%%%%%%%%%%%%%%%%%%% START EDITS FOR Q6 HERE %%%%%%%%%%%%%%%%%%%%%%
m = 0.027; % mass in kg

% Calculate the forces using the fit acceleration
% Recall: the variables that store the fit accelerations are at and an
Ft = m*at;
Fn = m*an;
%%%%%%%%%%%%%%%%%%%%%%%%%%%% END EDITS FOR Q6 HERE %%%%%%%%%%%%%%%%%%%%%%%%

%% Plot the forces in normal/tangential components
F_tn = [Ft, Fn];
directions = {'t', 'n'};

% Plot acceleration
figure(6)
for ii = 1:2
    % Plot each component of the acceleration on its own axis
    subplot(2,1,ii)
    plot(times, F_tn(:,ii), 'b')
    xlabel('Time [s]')
    ylabel(['F_' directions{ii}])
    ylim([-0.2 0.2])
end
subplot(2,1,1)
title('Forces in Tangential and Normal Coordinates')