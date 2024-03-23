classdef toy_problem<dynamic_sys
    
    methods
        function obj =  toy_problem(N_steps,ref_pts,sn)
            
            obj.name = 'toy_prob';
            obj.plotname = 'Toy Problem';
            obj.dimx = 1;
            obj.dimu = 1;
            
            % Bound of region of interest
            obj.LB = -[5;2];
            obj.UB = [5;2];
            
            % Bounds on control input
            obj.u_LB = -5;
            obj.u_UB = 5;
            obj.dt = 0.05; %0.1;
            
            
            obj.x0 = 5;
            
            if nargin>0
                obj.N_steps = N_steps;
            else
                obj.N_steps = 200;
            end
            if nargin>1
                obj.ref_pts = ref_pts;
            else
                n_A = 16;
                obj.ref_pts = gen_ref_pts(obj, n_A);
            end
            if nargin>2
                obj.sn = sn;
            else
                obj.sn = 0.05;
            end
            %obj.RMSE_grid = obj.LB + rand(obj.dimx+obj.dimu,N_RMSE).*(obj.UB-obj.LB);
            obj.prior = @(x,u) x;
        end
        
        function xplus = x_plus1(obj,x0,u)
            % Toy problem dynamics
           
            dx = @(t,x) 10*(sin(x) + atan(x) + u);
            [t, x_traj] = ode45(@(t,x) dx(t,x), [0 obj.dt], x0);
            xplus = x_traj(end,:)';
            %xplus = x0 + dx([],x0)*dt;
        end
    end
end