classdef DiscreteDynamicalSystem
    %DISCRETEDYNAMICALSYSTEM Class container discrete-time dynamical
    %systems
    
    properties
        nx
        nu
        hasVisualizer
        visualize;
    end
    
    methods
        function obj = DiscreteDynamicalSystem(nx, nu)
            obj.nx = nx;
            obj.nu = nu;
        end
        
        function obj = addVisualizer(obj, visualizer)
            obj.visualize = visualizer;
            obj.hasVisualizer = true;
        end
        
        function [xp] = dynamics(obj, xm, u, h)
            xp = xm;
        end
        
        function [t, x] = simulate(obj, h, N, x0, visualize, framerate)
           t(1) = 0;
           x(:,1) = x0;
           for i=1:N
              t(i+1) = i*h;
              x(:,i+1) = obj.dynamics(x(:,i),zeros(obj.nu,1),h);
           end
           
           if (nargin > 4) && visualize
               disp('waiting for visualization start')
               pause
               obj.visualizeTrajectory(t, x, framerate);
           end
        end
        
        function visualizeTrajectory(obj, t, x, framerate)
            if nargin < 4
                framerate = 1;
            end
            N = numel(t);
            tic
            obj.visualize(x(:,1));
            for i=(framerate+1):framerate:N
               pause(t(i)-t(i-framerate)-toc);
               tic
               obj.visualize(x(:,i));
            end
        end
    end
    
end

