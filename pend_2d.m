classdef pend_2d<dynamic_sys
    
    methods
        function obj =  pend_2d(N_steps,ref_pts,sn)
            
            obj.name = 'pendulum';
            obj.plotname = 'Pendulum';
            obj.dimx = 2;
            obj.dimu = 1;
            
            obj.LB = -[pi/2;5;2];
            obj.UB = [pi/2;5;2];
            
            obj.u_LB = -10;
            obj.u_UB = 10;
            obj.x0 = [pi; 0];
            
            if nargin>0
                obj.N_steps = N_steps;
            else
                obj.N_steps = 100;
            end
            if nargin>1
                obj.ref_pts = ref_pts;
            else
                n_A = 200;
                obj.ref_pts = gen_ref_pts(obj, n_A);
            end
            if nargin>2
                obj.sn = sn;
            else
                obj.sn = 0.05;
            end
            %obj.RMSE_grid = obj.LB + rand(obj.dimx+obj.dimu,N_RMSE).*(obj.UB-obj.LB);
            obj.prior = @(x,u) 0;
            obj.dt = 0.05;
        end
        
        function xplus = x_plus1(obj,x0,u)
            % Pendulum dynamics
            
            g = 9.81;
            m = 0.1;
            b = 0.05;
            l = 1;
            I = m*l^2;
            dx = @(t,x) [x(2); 1/I*(-b*x(2)-m*g*l*sin(x(1)) + u)];
            [t, x_traj] = ode45(@(t,x) dx(t,x), [0 obj.dt], x0);
            xplus = x_traj(end,:)';
            %xplus = x0 + dx([],x0)*dt;
        end
    end
end