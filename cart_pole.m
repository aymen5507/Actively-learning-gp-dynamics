classdef cart_pole<dynamic_sys
    
    methods
        function obj =  cart_pole(N_steps,ref_pts,sn)
            
            obj.name = 'cart_pole';
            obj.plotname = 'Cart-Pole';
            obj.dimx = 3; % 4
            obj.dimu = 1;
            obj.x0 = [0;pi;0];
            
            obj.LB = -[2; pi/4; 5;5]; %[0.1;0.5; pi/4; 0.01;5];
            obj.UB = [2; pi/4; 5;5]; %[0.1;0.5; pi/4; 0.01;5];
            
            obj.u_LB = -15;
            obj.u_UB = 15;
            if nargin>0
                obj.N_steps = N_steps;
            else
                obj.N_steps = 500;
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
            
            obj.dt = 0.05; %0.1;
            obj.prior = @(x,u) x; %+obj.dt*dx(obj,0,x,u,0);
            %[x(2,:);u/0.5;x(4,:);u/0.5*(sin(x(3))+cos(x(3)))];
            
            obj.gp_info = 3;
            obj.reward = @(x,u) 10 - (1-cos(x(3))) - 10e-5*u^2 ;
        end
        
        function xplus = x_plus1(obj,x0,u)
            % Cart-pole dynamics
            [t, x_traj] = ode45(@(t,x) dx(obj,t,x,u,1), [0 obj.dt], x0);
            xplus = x_traj(end,:)';
            %xplus = x0 + dx([],x0)*dt;
        end
        
        function dfdx = dx(obj,t,x,u,exact)
            g = 9.81;
            if exact
                m_pole = 0.1;
                m_cart = 0.5;
                l = 0.5; % Half the length of the pole
                mu_c = 0.05; % Friction coefficient of cart
                mu_p = 0.01; % Friction coefficient of pole
            else
                m_pole = 0.02;
                m_cart = 0.2;
                l = 0.1; % Half the length of the pole
                mu_c = 0; % Friction coefficient of cart
                mu_p = 0; % Friction coefficient of pole
            end
            % pos = x(1);
            dpos = x(1);
            theta = x(2);
            dtheta = x(3);
            
            sint = sin(theta);
            cost = cos(theta);
            
%             xdd = @(x,xd,theta,thetad,ddtheta)...
%                 (u+m_pole*l*(thetad^2*sin(theta)-ddtheta*cos(theta))-mu_c*sign(xd))/(m_cart+m_pole);
%             ddtheta = @(x,xd,theta,thetad) (g*sin(theta) ...
%                 +cos(theta)*(-u-m_pole*l*thetad^2*sin(theta)+mu_c*sign(xd))/(m_pole+m_cart)...
%                 -mu_p*thetad/m_pole/l)/l/(4/3 - m_pole*cos(theta)^2/(m_pole+m_cart));
%             dfdx = [x(2); ...
%                 xdd(x(1),x(2),x(3),x(4),ddtheta(x(1),x(2),x(3),x(4)));...
%                 x(4); ddtheta(x(1),x(2),x(3),x(4))];
            ddtheta = (g*sint +cost*(-u-m_pole*l*dtheta^2*sint+mu_c*sign(dpos))/(m_pole+m_cart)...
                -mu_p*dtheta/m_pole/l)/l/(4/3 - m_pole*cost^2/(m_pole+m_cart));
            ddpos = (u+m_pole*l*(dtheta^2*sint-ddtheta*cost)-mu_c*sign(dpos))/(m_cart+m_pole);
            dfdx = [ddpos; dtheta; ddtheta];
                %x(4); ddtheta(x(1),x(2),x(3),x(4))];
            
        end
    end
end