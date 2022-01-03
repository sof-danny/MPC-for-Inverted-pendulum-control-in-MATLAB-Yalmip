clear
clc
close all 
warning('off','all');

% Check out paper by Chalupa. His has different state order defined. 

% system parameters 
M = 5; m = 30; b = 1; L = 1.5; J = 1/12*m*(2*L)^2; g = 9.81; th_e = pi;

% matrices needed to form state-space
N = [M+m,m*L*cos(th_e);m*L*cos(th_e),J+m*L^2];
%A =[M+m,m*l*cos(x(2));m*l*cos(x(2)),J+m*l^2];
S = [-b,0;0,0];
T = [0,0;0,-m*g*L*cos(th_e)];
%B= [u + m*l*x(4)^2*sin(x(2))-b*x(3);-m*g*l*sin(x(2))]
W = [1;0];

% form the state-space matrices; this is for an inverted pendulum where
% the states are position, velocity, pitch angle, and angular velocity

%%%Position, angle, and then rates
A = [0,0,1,0;0,0,0,1;N\T,N\S];
B = [0;0;N\W];
C = [0,1,0,0];
D = [0];

% discretize the system
T = 0.1; tf = 20; Ts = T;
A = expm(A*T);
B = T*B; D = 0; P = [M,m,L,b,J,g];

x0 = [0;th_e - 5*pi/180;0;0]; % perturb the pendulum from vertical position
K = zeros(1,4); % zero gains for uncontrolled case

%tuc = rand(100,1);
%xuc = rand(100,4) ;
[tuc,xuc] = sim('sim_disc'); clear time; % uncontrolled baseline case
[tc,xc] = sim('inv_pend_sim_with_MPC_nonlin_1'); % run with MPC controller
 
% plot results (note that 180 degrees is the inverted position)
figure
subplot(211)
stairs(tuc,xuc(:,1),'r','LineWidth',2)
hold on
subplot(212)
stairs(tuc,xuc(:,2)*180/pi,'r','LineWidth',2)
hold on
subplot(211)
stairs(tc,xc(:,1),'b','LineWidth',2)
legend('Open-loop','MPC Ctrller','Location','Best')
legend('boxoff')
xlabel('Time (s)')
ylabel('x (m)')
hold on
%plot(time,zeros(numel(time)),'--k')
set(gca,'fontsize',16)
subplot(212)
stairs(tc,xc(:,2)*180/pi,'b','LineWidth',2)
xlabel('Time (sec)')
ylabel('\theta (\circ)')
hold on
plot(time,180*ones(numel(time)),'--k')
set(gca,'fontsize',16)

figure
subplot(211)
stairs(tc,xc(:,1),'LineWidth',2)
legend('MPC Ctrller','Location','Best')
legend('boxoff')
xlabel('Time (s)')
ylabel('x (m)')
hold on
%plot(time,zeros(numel(time)),'--k')
set(gca,'fontsize',16)
subplot(212)
stairs(tc,xc(:,2)*180/pi,'LineWidth',2)
xlabel('Time (sec)')
ylabel('\theta (\circ)')
hold on
plot(time,180*ones(numel(time)),'--k')
set(gca,'fontsize',16)
