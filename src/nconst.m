function [ineqvalue, eqvalue] = nconst(eta, prob_info)

N = prob_info.N;
Nx = prob_info.Nx;
Nu = prob_info.Nu;
T = prob_info.T;
x0 = prob_info.x0;

delta = T/N;

x = reshape(eta(1:N*Nx,1), Nx, N);

x = [x0 x];

u = reshape(eta((N*Nx+1):(N*Nx+N*Nu)), Nu, N);

eqvalue = zeros(N*Nx,1);

for index=1:N
    eqvalue( ((index-1)*Nx+1):index*Nx,1) = x(:,index+1) - x(:,index) - delta*sys_h(x(:,index), u(:,index)); 
end 

ineqvalue = zeros(N,1);

r = prob_info.radii;
c1 = prob_info.centres(1,1); 
c2 = prob_info.centres(1,2);


for index= 1:N
    ineqvalue(index,1) = r^2 - (x(1,index+1)-c1)^2 - (x(2,index+1)-c2)^2;
end





