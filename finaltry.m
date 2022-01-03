function uout = finaltry(currentx,currentr,t,Ts)

% this function uses Yalmip to solve the MPC optimization... here you can
% build in your constraints on your states, inputs, etc.  I have a
% constraint saying x_k+1 = A*x_k + B*u_k and I have applied a saturation 
% limit to the control effort.

persistent Controller

if t == 0 % set up the problem at time t = 0;
% Compute discrete-time dynamics
M = 5; m = 30; b = 1; L = 1.5; J = 1/12*m*(2*L)^2; g = 9.81; th_e = pi;
N = [M+m,m*L*cos(th_e);m*L*cos(th_e),J+m*L^2];
S = [-b,0;0,0];
T = [0,0;0,-m*g*L*cos(th_e)];
W = [1;0];
A = [0,0,1,0;0,0,0,1;N\T,N\S];
B = [0;0;N\W];
C = [0,1,0,0];
D = [0];

A = expm(A*Ts);
B = Ts*B; C = [0,1,0,0]; 

Q = 1000*eye(4);
R = 1/100;
N = 25;

    % Avoid explosion of internally defined variables in YALMIP
    yalmip('clear')

    % Setup the optimization problem
    nu = 1; %number of input 
    ny = 1; %number of outputs 
    nx = 4; %number of states (4 states, SISO) 
    
    u = sdpvar(repmat(nu,1,N),repmat(1,1,N));
    x = sdpvar(repmat(nx,1,N+1),repmat(1,1,N+1));
%     sdpvar r
    
    % Define simple standard MPC controller
    % Current state is known so we replace this
    constraints = [];
    objective = 0;
    
    for k = 1:N % N steps in the MPC horizon
        objective = objective + x{k}'*Q*x{k} + u{k}'*R*u{k}; % put your obj. function here
        constraints = [constraints, x{k+1} == A*x{k}+B*u{k}]; % add any constraints in this manner
        constraints = [constraints, -50 <= u{k}<= 50];
    end

    % Define an optimizer object which solves the problem for a particular
    % initial state and reference
%     Controller = optimizer(constraints,objective,[],{x{1},r},u{1});
        ops = sdpsettings('solver','quadprog')
    Controller = optimizer(constraints,objective,ops,{x{1}},u{1});
    % And use it here too
%     uout = Controller{{currentx,currentr}};
    uout = Controller{{currentx}};
else    
    % Almost no overhead
%     uout = Controller{{currentx,currentr}};
    uout = Controller{{currentx}};

end

end