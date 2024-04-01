load('cart_u')
model=cart_pole;
xdim=model.dimx;

clear x_hat
load('cart_pole_iter_1.mat') %saved gp model
load('cart_x_true.mat')
x_hat(1,:)=model.x0;
for i=2:length(u)
    for j=1:xdim
        x_hat(i,j)=predict(gpmodel{j},[x_hat(i-1,:) u(i-1)]);      
    end
end
save('cart_x_hat.mat', 'x_hat');
immse(x_hat,x)
