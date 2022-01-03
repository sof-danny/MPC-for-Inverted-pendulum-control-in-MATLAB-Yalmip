% S-function to describe the dynamics of the simple pendulum
% Input is a force to the cart and output is pendulum angle

function [sys,x0,str,ts] = sfunc_disc(t,x,u,flag,x0,P,K) 
% t is time
% x is state
% u is input
% flag is a calling argument used by Simulink.
% The value of flag determines what Simulink wants to be executed.

switch flag
case 0 % Initialization
[sys,str,ts]=mdlInitializeSizes;
case 1 % Compute xdot
sys=mdlDerivatives(t,x,u,P,K);
case 2 % Not needed for continuous-time systems
case 3 % Compute output
sys = mdlOutputs(t,x,u,P,K);

case 4 % Not needed for continuous-time systems
case 9 % Not needed here
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% mdlInitializeSizes
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [sys,str,ts]=mdlInitializeSizes

% Create the sizes structure
sizes=simsizes;
sizes.NumContStates = 4; % Set number of continuous-time state variables
sizes.NumDiscStates = 0;
sizes.NumOutputs = 4; % Set number of output variables
sizes.NumInputs = 1; % Set number of input variables
sizes.DirFeedthrough = 0;
sizes.NumSampleTimes = 1; % Need at least one sample time
sys = simsizes(sizes);


str=[]; % str is always an empty matrix
ts=[0 0]; % ts must be a matrix of at least one row and two columns

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% mdlDerivatives:  computes the state derivatives
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function sys = mdlDerivatives(t,x,u,P,K)

% parameters of the model

M = P(1); m = P(2); l = P(3); b = P(4); J = P(5); g = P(6);

% Compute xdot based on (t,x,u) and set it equal to sys

sys(1:2) = x(3:4);

A = [M+m,m*l*cos(x(2));m*l*cos(x(2)),J+m*l^2];
B = [u + m*l*x(4)^2*sin(x(2))-b*x(3);-m*g*l*sin(x(2))];

sys(3:4) = A\B;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% mdlOutput
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function sys = mdlOutputs(t,x,u,P,K)

% Compute output based on (t,x,u) and set it equal to sys
sys = [x(1);x(2);x(3);x(4)];
