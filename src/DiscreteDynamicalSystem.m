classdef DiscreteDynamicalSystem
    %DISCRETEDYNAMICALSYSTEM Class container discrete-time dynamical
    %systems
    
    properties
        nx
        nu
    end
    
    methods
        function obj = DiscreteDynamicalSystem(nx, nu)
            obj.nx = nx;
            obj.nu = nu;
        end
        
        function [x_ip1] = dynamics(obj, x, u)
            x_ip1 = x;
        end
    end
    
end

