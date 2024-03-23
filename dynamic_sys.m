classdef dynamic_sys<matlab.mixin.Copyable
   
    properties
        name;
        x0;
        plotname;
        N_steps;
        ref_pts;
        dimx;
        dimu;
        sn;
        LB;
        UB;
        u_LB;
        u_UB;
        prior;
        gp_info;
        reward;
        dt;
    end
    
    methods
        function A = gen_ref_pts(obj, n_A)
            % Generate grid of uniformly spaced reference points based on interval 
            %I_LB-I_UB with total desired number of points n_A
            
            
            I_LB = obj.LB;
            I_UB = obj.UB;
            n_dis = zeros(obj.dimx,1);
            dis = cell(obj.dimx,1);
            I = sum(I_UB-I_LB);
            %obj.x0 = 0;
            
            try
                dimxaug = length(I_UB);
            catch
                dimxaug = obj.dimx+obj.dimu;
            end
            for i = 1:dimxaug
                n_dis(i) = ceil((n_A*(I_UB(i)-I_LB(i))/I)^(1/dimxaug));
                dis{i} = linspace(I_LB(i),I_UB(i),n_dis(i))';
            end
            if prod(n_dis) ~= n_A
                warning('The number of reference points A is higher than the desired quantity n_A.')
            end
            combinations = cell(1, numel(dis));
            [combinations{:}] = ndgrid(dis{:});
            combinations = cellfun(@(x) x(:), combinations,'uniformoutput',false); 
            A = [combinations{:}]';

        end
        
        function [X_data,Xplus_data] = gen_data(obj,N_data,u)
            X_data = zeros(obj.dimx+obj.dimu,N_data);
            Xplus_data = zeros(obj.dimx,N_data);
            xt = obj.x0;
            for i=1:N_data
                ut = u(xt);
                X_data(:,i) = [xt; ut];
                Xplus_data(:,i) = x_plus1(obj,xt,ut) + obj.sn*randn;
                xt = x_plus1(obj,xt,ut);
            end
        end
        
        function [X_new, Y_new] = filter_data(obj,X, Y)
            is_active = linspace(1,size(X,2),size(X,2));
            for i=size(X,2):-1:1
                if any(X(:,i) < 2*obj.LB) || any(X(:,i) > 2*obj.UB)
                    is_active(i) = [];
                end
            end
            if length(is_active)>2
                X_new = X(:,is_active);
                Y_new = Y(:,is_active);
            else
                X_new = X;
                Y_new = Y;
            end
            % The following lines disable filtering. Comment to enable
            % filtering
            X_new = X;
            Y_new = Y;
        end
        
        function error_grid = RMSE_grid(obj,N_RMSE)
            if nargin<2
                N_RMSE = 1000;
            end
            error_grid = obj.LB + rand(obj.dimx+obj.dimu,N_RMSE).*(obj.UB-obj.LB);
        end
    end
end