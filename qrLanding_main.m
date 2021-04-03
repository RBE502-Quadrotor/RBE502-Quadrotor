clear all; clc; close all;

% change current folder to wherever your git repos are;
% this will include the templates folder, with the code the professor
% provided us
addpath(".\templates\");

% initial conditions (State Vector)
z0 = [5;5;5; 0;0;0; 0;0;0; 0;0;0]; % Current Position to go to landing position

% External Forces Vector
rng(1)
r = rand(3,1);

% External Moment Vector
rng(2)
n = rand(3,1);

% r = [0; 0; 0];  % External Forces
% n = [0; 0; 0];  % Moment Vector
u0 = [1; 0.9; 1.9; 1.5]; % rotor/motor inputs (?)

qr = QuadrotorClass(z0, r, n, u0);

% intruder parameters
u_intruder = [1; 0.9; 1.9; 1.5];
intruder = QuadrotorClass(z0, zeros(3,1), zeros(3,1), u_intruder);

% Quadrotor constants
% m = qr.m;
% g = qr.g;
% I = qr.I;
% mu = qr.mu;
% sigma = qr.sigma;
% l = qr.l;

%System Dynamics 
% A = zeros(12);
% A(1:6,7:end) = eye(6);
% A(7,5) = g;
% A(8,4) = -g;

A =[0, 0, 0,  0,    0,    0, 1, 0, 0, 0, 0, 0;
    0, 0, 0,  0,    0,    0, 0, 1, 0, 0, 0, 0;
    0, 0, 0,  0,    0,    0, 0, 0, 1, 0, 0, 0;
    0, 0, 0,  0,    0,    0, 0, 0, 0, 1, 0, 0;
    0, 0, 0,  0,    0,    0, 0, 0, 0, 0, 1, 0;
    0, 0, 0,  0,    0,    0, 0, 0, 0, 0, 0, 1;
    0, 0, 0,  0,    qr.g, 0, 0, 0, 0, 0, 0, 0;
    0, 0, 0, -qr.g, 0,    0, 0, 0, 0, 0, 0, 0;
    0, 0, 0,  0,    0,    0, 0, 0, 0, 0, 0, 0;
    0, 0, 0,  0,    0,    0, 0, 0, 0, 0, 0, 0;
    0, 0, 0,  0,    0,    0, 0, 0, 0, 0, 0, 0;
    0, 0, 0,  0,    0,    0, 0, 0, 0, 0, 0, 0];

B = [0,                0,                 0,                0; 
     0,                0,                 0,                0; 
     0,                0,                 0,                0; 
     0,                0,                 0,                0; 
     0,                0,                 0,                0;  
     0,                0,                 0,                0; 
     0,                0,                 0,                0; 
     0,                0,                 0,                0;
     1/qr.m,           1/qr.m,            1/qr.m,           1/qr.m;    
     0,                qr.l/qr.I(1),      0,                -qr.l/qr.I(1); 
     -qr.l/qr.I(2),    0,                 qr.l/qr.I(2),     0;
     qr.sigma/qr.I(3), -qr.sigma/qr.I(3), qr.sigma/qr.I(3), -qr.sigma/qr.I(3)];

% C = [ 1 0 0 0 0 0 0 0 0 0 0 0; 
%       0 1 0 0 0 0 0 0 0 0 0 0;
%       0 0 1 0 0 0 0 0 0 0 0 0;
%       0 0 0 0 0 0 1 0 0 0 0 0;
%       0 0 0 0 0 0 0 1 0 0 0 0;
%       0 0 0 0 0 0 0 0 1 0 0 0];

% D = [0 0 0 0; 0 0 0 0; 0 0 0 0; 0 0 0 0; 0 0 0 0; 0 0 0 0;];

% tuning performance cost (judged by state vector; affected by state error?)
Q = [1 0 0 0 0 0 0 0 0 0 0 0;   % x error
    0 1 0 0 0 0 0 0 0 0 0 0;    % y error
    0 0 1 0 0 0 0 0 0 0 0 0;    % z error
    0 0 0 1 0 0 0 0 0 0 0 0;    % angular rotation (theta 1) error
    0 0 0 0 1 0 0 0 0 0 0 0;    % angular rotation (theta 2) error
    0 0 0 0 0 1 0 0 0 0 0 0;    % angular rotation (theta 3) error
    0 0 0 0 0 0 2 0 0 0 0 0;    % rate of translation (x) error
    0 0 0 0 0 0 0 2 0 0 0 0;    % rate of translation (y) error
    0 0 0 0 0 0 0 0 2 0 0 0;    % rate of translation (z) error
    0 0 0 0 0 0 0 0 0 2 0 0;    % rate of rotation (theta 1) error
    0 0 0 0 0 0 0 0 0 0 2 0;    % rate of rotation (theta 2) error
    0 0 0 0 0 0 0 0 0 0 0 2];   % rate of rotation (theta 3) error


% Q = eye(12);
% R = 10*eye(4);

% tuning actuator cost (judged by input gains; affects acceleration allowed or energy expended for maneuver)
R = [1 0 0 0;       % x dot
    0 1 0 0;        % alpha dot
    0 0 1 0;        % v dot
    0 0 0 1];       % omega dot

K = lqr(A,B,Q,R);

% closed loop system 

%sys = ss((A -B*K), B, C, D);

u0  = ones(4,1)*qr.m*qr.g/4;

% u=@(z) K*(zd - z) + u0;

t = linspace(0, 10, 200);

% State values for landing back at nest
z_hover = [0;0;1; 0;0;0; 0;0;0; 0;0;0]; % hover point above the nest
z_nest = [0;0;0; 0;0;0; 0;0;0; 0;0;0]; % position and state for nest

u1=@(t,z) K*(z_hover - z) + u0;
u2=@(t,z) K*(z_nest - z) + u0;

[t1, z1] = ode45(@(t,z) quadrotor(t,z,u1,qr.p,qr.r,qr.n), t, z0);
[t2, z2] = ode45(@(t,z) quadrotor(t,z,u2,qr.p,qr.r,qr.n), t, z1(end,:));

t = [t1;t1(end)+t2];
z = [z1;z2];

% Plot states for landing
figure
qr.plotResults(t, z);
subtitle('Return to Nest');


%% Set Up Animation

animation_fig = figure;
title('Animated Simulation');

airspace_box_length = 10;

animation_axes = axes('Parent', animation_fig,...
    'NextPlot','add','DataAspectRatio',[1 1 1],...
    'Xlim',airspace_box_length*[-1.0 1.0],...
    'Ylim',airspace_box_length*[-1.0 1.0],...
    'Zlim',airspace_box_length*[0 1],...
    'box','on','Xgrid','on','Ygrid','on','Zgrid','on',...
    'TickLabelInterpreter','LaTeX','FontSize',14);

view(animation_axes, 3);

N = 10;
QQ = linspace(0,2*pi,N)';
circle = 0.3*qr.l*[cos(QQ) sin(QQ) zeros(N,1)];
loc = qr.l*[1 0 0; 0 1 0; -1 0 0; 0 -1 0];


silhouette = plot3(0,0,0, '--', 'Color', 0.5*[1 1 1], 'LineWidth', 1 ,...
    'Parent', animation_axes);


body = plot3(0,0,0, 'Color',lines(1), 'LineWidth', 2,...
        'Parent', animation_axes);

% Define intruder color and intruder body
intruder_Color = lines(2);
intruder_Color = intruder_Color(2,:);
intruder_Body = plot3(2,2,2, 'Color',intruder_Color, 'LineWidth', 2,...
        'Parent', animation_axes);

% Positions of rotors around center of mass
for i=1:4
    rotor(i) = plot3(0,0,0, 'Color', lines(1), 'LineWidth', 2,...
        'Parent', animation_axes);
    intruder_Rotor(i) = plot3(0,0,0, 'Color', intruder_Color, 'LineWidth', 2,...
        'Parent', animation_axes);
end

% Fake state matrix for captured intruder quadcopter, .25 meters below
% quadcoper ( for animation purposes )
% intruder_z = [ones(length(t),3)*5, zeros(length(t),3)];
intruder_z = z + [zeros(length(z),2),ones(length(z),1)*-.25,zeros(length(z),9)];

tic;

%% Run Simulation
for k=1:length(t)

    % Rotation matrix for quadcopter
    R = qr.quadrotorRotation(z(k,4), z(k,5), z(k,6));
    
    % Enemy Rotation Matrix
    intruder_R = intruder.quadrotorRotation(intruder_z(k,4), intruder_z(k,5), intruder_z(k,6));

    % Update pose of each of the 4 rotors
    for i=1:4
        % Update pose for quadrotor motors
        ctr(i,:) = z(k,1:3) + loc(i,:)*R';
        pose = ones(N,1)*z(k,1:3) + (ones(N,1)*loc(i,:) + circle)*R';
        set(rotor(i), 'XData', pose(:,1), 'YData', pose(:,2),  'ZData', pose(:,3) );
        
        % Update pose for intruder quadrotor
        intruder_Ctr(i,:) = intruder_z(k,1:3) + loc(i,:)*intruder_R';
        intruder_Pose = ones(N,1)*intruder_z(k,1:3) + (ones(N,1)*loc(i,:) + circle)*intruder_R';
        set(intruder_Rotor(i), 'XData', intruder_Pose(:,1), 'YData', intruder_Pose(:,2),  'ZData', intruder_Pose(:,3) );
    end
    
    % Animate silhouette of X-Y-Z position for quadrotor
    set(silhouette,'XData', [0, z(k,1), z(k,1), z(k,1)],...
        'YData', [0, 0, z(k,2), z(k,2)],...
        'ZData', [0, 0, 0, z(k,3)]);
    
    % Animate Quadrotor
    set(body, 'XData', [ctr([1 3],1); NaN; ctr([2 4],1)], ...
        'YData', [ctr([1 3],2); NaN; ctr([2 4],2)],...
        'ZData', [ctr([1 3],3); NaN; ctr([2 4],3)] );
    
    % Animate Enemy
    set(intruder_Body, 'XData', [intruder_Ctr([1 3],1); NaN; intruder_Ctr([2 4],1)], ...
        'YData', [intruder_Ctr([1 3],2); NaN; intruder_Ctr([2 4],2)],...
        'ZData', [intruder_Ctr([1 3],3); NaN; intruder_Ctr([2 4],3)] );
    pause(t(k)-toc);
    pause(0.01);
end
