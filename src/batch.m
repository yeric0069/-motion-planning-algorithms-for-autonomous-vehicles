N = 64;                     % Number of discretisation intervals 
Nx = 5;                     % Number of state variables 
Nu = 2;                     % Number of input variables 
T = 10;                     % Horizon length
x0 = [0; 0; pi/4; 0; 0];    % Initual state 
goal = [10; 10; 0];         % Goal pose 

% prob_info is a structure containing problem related information, which
% will be passed to sub-functions whenever it's necessary 

prob_info = struct('T',  T,  'N',    N, 'Nu', Nu, 'Nx', Nx, 'x0', x0, 'xT', goal, 'centres', [5 5], 'radii', 2);


% preparing initial optimisation variables for the solver 

init_control = 0.01 + 0.02*rand(Nu,N);
init_state = zeros(Nx, N);

delta = T/N; 
init_state(:,1) = x0 + delta*sys_h(x0,init_control(:,1), prob_info);

for idx = 2:N
    init_state(:,idx) = init_state(:,idx-1) + delta*sys_h(init_state(:,idx-1), init_control(:,idx), prob_info);
end 

% eta is the vector that includes all the optimisation variables, x(1),...,
% x(N) and u(0),..., u(N-1)

eta = [init_state(:) ; init_control(:)];

% linear inequality and equality constraints, we don't have any of these in
% our problem, so we define them as empty variables. 

A = [];
b = [];
Aeq = [];
beq = [];

% upper and lower bounds of decision variables 

lb_control = -inf(Nu,N);
ub_control = inf(Nu,N);

lb_state = -inf(Nx, N);
ub_state = inf(Nx,N);

lb_control(1,: ) = -0.5;    % steering angle lower bound, rad
lb_control(2,: ) = -2;      % acceleration lower bound, m/s^2 

ub_control(1,: ) = 0.5;     % steering angle upper bound, rad
ub_control(2,: ) = 1;       % acceleration upper bound, m/s^2 

lb = [lb_state(:); lb_control(:)];
ub = [ub_state(:); ub_control(:)];

% function handles for cost and constraints 

fun = @(eta) cost(eta, prob_info);
nonlcon = @(eta) nconst(eta, prob_info);

options = optimoptions('fmincon','Display','notify-detailed','MaxIterations', 1e4, 'MaxFunctionEvaluations', 1e6);

% Solve the constrained nonlinear optimisation problem using fmincon
[eta_star, cost_star, exitflag, output] = fmincon(fun,eta,A,b,Aeq,beq,lb,ub,nonlcon,options);

xresult = [x0 reshape(eta_star(1:N*Nx,1), Nx, N)];
xinit = [x0 reshape(eta(1:N*Nx,1), Nx, N)];

uresult = reshape(eta_star((N*Nx+1):(N*Nx+N*Nu)), Nu, N);

figure, hold
plot(xresult(1,:), xresult(2,:))
plot(xinit(1,:), xinit(2,:), 'r:')
viscircles(prob_info.centres, prob_info.radii);
axis equal

figure, hold
plot(uresult(1,:))
plot(uresult(2,:), 'r')
ylabel('Steering angle input, rad, Acceleration (red), m/s^2')

figure, hold
plot(xresult(3,:)*180/pi)
ylabel('Vehicle Heading, deg')

figure, hold
plot(xresult(4,:))
ylabel('Forward Speed, m/s')
