%% Load data
load drone_data.mat

%% Define parameters
dt = mean(diff(times)); % estimate dt to be the average difference between time measurements
m = 0.027; % mass [kg]
g = 9.81; % gravitational constant [m/s^2]

%% Load velocity and acceleration fit data
% Note, we are only going to use the fit data, not the smoothed data
[~, ~, v_fit, a_fit, ~, ~, at_fit, an_fit, ~, ~, f] = p2_solution(times, pos);
pos_fit = [feval(f{1}, times), feval(f{2}, times), feval(f{3}, times)];

%% Plot 3d position
figure(1)
comet3(pos_fit(:,1), pos_fit(:,2), pos_fit(:,3))
axis image
axis vis3d
xlabel('x')
ylabel('y')
zlabel('z')

%% Plot individual components of the position
coordinates = {'x', 'y', 'z'};
figure(2)
for i = 1:3
    % Plot each component of the position on its own axis
    subplot(3,1,i)
    plot(times, pos(:,i), 'ko', times, pos_fit(:,i), 'r')
    xlabel('Time [s]')
    ylabel(coordinates{i})
end
subplot(3,1,1)
title('Position')
legend('Raw data', 'Fit', 'Location', 'Best')
drawnow

%% Calculate kinetic and potential energy
%%%%%%%%%%%%%%%%%%%%%%%%%%%% START EDITS FOR Q1 HERE %%%%%%%%%%%%%%%%%%%%%%
% Calculate the potential energy, V
V = m*g*pos_fit(:,3);

% Calculate the kinetic energy, T
T = 0.5 * m * sum(v_fit.^2, 2);
%%%%%%%%%%%%%%%%%%%%%%%%%%%% END EDITS FOR Q1 HERE %%%%%%%%%%%%%%%%%%%%%%%%
E = V + T; % Collect total energy

%% Plot energy
figure(3)
plot(times, V, times, T, times, E)
legend('U', 'T', 'Total energy')
xlabel('Time [s]')
ylabel('Energy [J]')
drawnow

%% Calculate the non-conservative forces in rectangular coordinates
%%%%%%%%%%%%%%%%%%%%%%%%%%%% START EDITS FOR Q2 HERE %%%%%%%%%%%%%%%%%%%%%%
Fx_fit = m*a_fit(:,1);
Fy_fit = m*a_fit(:,2);
Fz_fit = m*a_fit(:,3);

% Written question: What are the non-conservative forces acting on the
% drone during flight? (Please make sure your answer is in a comment.)
% The non-conservative forces acting on the drone as it flies through its
% route is from the air resistance it experiences.
%%%%%%%%%%%%%%%%%%%%%%%%%%%% END EDITS FOR Q2 HERE %%%%%%%%%%%%%%%%%%%%%%%%
F_fit = [Fx_fit, Fy_fit, Fz_fit]; % Collect forces into single matrix

%% Calculate the power from non-conservative forces
%%%%%%%%%%%%%%%%%%%%%%%%%%%% START EDITS FOR Q3 HERE %%%%%%%%%%%%%%%%%%%%%%.
P = sum(F_fit .* v_fit, 2);
%%%%%%%%%%%%%%%%%%%%%%%%%%%% END EDITS FOR Q3 HERE %%%%%%%%%%%%%%%%%%%%%%%%

%% Calculate the work done by non-conservative forces in two different ways
%%%%%%%%%%%%%%%%%%%%%%%%%%%% START EDITS FOR Q4 HERE %%%%%%%%%%%%%%%%%%%%%%
% Work calculated as the integral of power with respect to time.
W1 = cumtrapz(times, P);

% Work calculated as the difference in energy from the initial state.
W2 = E - E(1);
%%%%%%%%%%%%%%%%%%%%%%%%%%%% END EDITS FOR Q4 HERE %%%%%%%%%%%%%%%%%%%%%%%%

%% Plot the power and work
figure(4)
subplot(2,1,1)
plot(times, 1000*P)
xlabel('Time [s]')
ylabel('Power [mW]')

subplot(2,1,2)
plot(times, W1, 'ko', times, W2, 'r-')
xlabel('Time [s]')
ylabel('Work [J]')
legend('W1', 'W2')
drawnow

%%%%%%%%%%%%%%%%%%%%%%%%%%%% START EDITS FOR Q5 HERE %%%%%%%%%%%%%%%%%%%%%%
% Written question: Discuss the trends that you see in the input power
% and work due to the non-conservative forces. For example, what do you 
% think the dominant force is at various times throughout the flight?
% (Please make sure your answer is in a comment.)

% After anaylzing the data, you can see that drones flight path varies
% frequently as it flies in a figure 8 pattern, so naturally the
% non-conservative forces and their strengths will vary as the drone
% progresses through its flight. If you look at the power and work graphs,
% you see these fluctuations in a more quantitative manner. You see there
% are sharp peaks in the power graph, which when correlated with peaks in
% the work graph can show that there may have been moments of strong
% turbulence or air resistance which resulted in these data points. These
% points can help to show exactly how these nonconservative forces
% fluctauate, and when they are at their strongest throughout the flight
% time. This allows us to assume that likely the dominant force is likely
% from strong gusts of wind or, more generally, air resistance.
%%%%%%%%%%%%%%%%%%%%%%%%%%%% END EDITS FOR Q5 HERE %%%%%%%%%%%%%%%%%%%%%%%%
