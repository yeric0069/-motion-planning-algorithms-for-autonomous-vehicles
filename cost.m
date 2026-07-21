function value = cost(eta, prob_info)

N = prob_info.N;
Nx = prob_info.Nx;
x0 = prob_info.x0;

x = reshape(eta(1:N*Nx,1), Nx, N);

x = [x0 x];

value = (x(1,N+1)-prob_info.xT(1,1))^2 + (x(2,N+1)-prob_info.xT(2,1))^2 + (x(3,N+1)-prob_info.xT(3,1))^2 + x(4,N+1)^2 + x(5,N+1);
