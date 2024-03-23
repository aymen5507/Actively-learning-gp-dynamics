clear
clear global
%% system model
global gpmodel old_state xdim
model=cart_pole; % toy_problem pend_2d cart_pole
xdim=model.dimx;
udim=model.dimu;
x0=model.x0;

ulb=model.u_LB; %upperbound of control input
uub=model.u_UB; %lowerbound of control input

initial_size=5;
iter=100;
sample_size=initial_size+iter; %initial traning set size

trials=10;
epsilon_k = -0.05 + (0.1)*rand(xdim,sample_size); %gaussian observation noise

%% generate validation set
test_num=1000;
random_state = rand(test_num, xdim);
random_input=ulb + (uub-ulb)*rand(test_num,udim);
random_samples=[random_state, random_input];

%% generate initial traning set
for j=1:trials
%seed = 9;
%rng(seed);
initial_inputs=ulb + (uub-ulb)*rand(sample_size,1);

state_before = x0;
for i=1:sample_size
    state_after(:,i)=model.x_plus1(state_before,initial_inputs(i));
    state_before=state_after(:,i);
end

state_observed=state_after+epsilon_k;
state_observed=state_observed';

x=[x0, state_after(:,1:end-1)];
x=[x;reshape(initial_inputs,[1,sample_size])];
x=x';

%% fit gp model
al_x=x(1:initial_size,:);
al_y=state_observed(1:initial_size,:);
fit_gpmodel(al_x,al_y);


%% Informative control generation
global N
N=15; %horizon length
options = optimoptions(@fmincon,'Algorithm','interior-point','MaxIterations',1500, 'TolCon', 1e-20, 'TolFun', 1e-20, 'MaxFunctionEvaluations', 5000);
lb = ulb * ones(N, 1);  
ub = uub * ones(N, 1);  


for k=1:iter
u_optimal = fmincon(@entropy_cal,ulb*ones(1, N), [], [], [], [], lb, ub, [], []);
[al_x,al_y]=utilize_control(al_x,al_y,u_optimal(1),model);
fit_gpmodel(al_x,al_y);
MSE(j,k)=test_result(model,random_samples,test_num) %calculate the MSE of the current model
%MSE(i)=test_result(model,[state_after' random_input],test_num) %calculate the MSE of the current model
end
save([model.name '_trial_' num2str(j)],'gpmodel')
for k=1:iter
fit_gpmodel(x(1:k+initial_size,:),state_observed(1:k+initial_size,:));
MSE_no_al(j,k)=test_result(model,random_samples,test_num);
end

% fit_gpmodel(x,state_observed);
% MSE_no_al(j)=test_result(model,random_samples,test_num);
end
for k=1:iter

end
MSE_mean=mean(MSE);
figure_configuration_IEEE_standard;
plot(linspace(1,iter,iter),MSE_mean,'color','#4DBEEE');
hold on;
plot(linspace(1,iter,iter),mean(MSE_no_al),'--r')
xlabel('iterations');
ylabel('MSE');
title('active learning GP (rec)')
legend('active learning GP','random sampling','Location','northeast')
%% function handle of the cost function
function cost = entropy_cal(u)

global old_state gpmodel N xdim;

  cost=0;
  last_state=old_state;
  for i=1:N
    for j=1:xdim
       [~,ysd(j)]=predict(gpmodel{j},[last_state u(i)]);
    end
    entropy=0.5*log(2*pi*exp(1)*(norm(ysd)^2) / 4);
    cost=cost+entropy;
    for k=1:xdim
    last_state(k)=predict(gpmodel{k},[last_state u(i)]);
    
    end
  end
  cost=-cost;
end
%% function for add in the sample to the traning set
function [new_x,new_y]=utilize_control(x,y,u,model)
new_x=[x;y(end,:) u];

next_state=model.x_plus1(y(end,:), u) ;
epsilon_k = -0.05 + (0.1)*rand(size(next_state'));
next_state=next_state'+epsilon_k;

new_y=[y;next_state];

end

%% train gp model
function fit_gpmodel(x,y)
global gpmodel xdim old_state;
for i=1:xdim
gpmodel{i}=fitrgp(x,y(:,i));
end
old_state=y(end,:);
end

%% calculate the MSE of gp model
function MSE=test_result(model,random_samples,test_num)
global gpmodel xdim;

for i=1:test_num
    true_results(:,i)=model.x_plus1(random_samples(i,1:end-1),random_samples(i,end));
    for j=1:xdim
    predicted_y(i,j)=predict(gpmodel{j},random_samples(i,:));
    end
end

MSE=immse(true_results, predicted_y');
end