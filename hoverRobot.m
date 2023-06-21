classdef hoverRobot
    %HOVERROBOT Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (Constant)
        rho = 1260 % Density of fluid, [kg/m^3]
        m = 0.15 % Mass of robot, [kg]
        mu = 1 % Viscosity of fluid, [Ns/m]

        lambda = 0.0046 % Pitch of cylinder thread, [m]
        r = 0.025 % Radius of cylinder, [m]
        h0 = 0.060 % Initial height of the robot, [m]
        g = -9.81 % Gravitational acceleration, [m/s^2]
    end
    properties  
        tspan
        x0
    end
    
    methods
        
        function dxdt = stateSpaceModel(obj,t,x)
            x1 = x(1); % Displacement
            x2 = x(2); % Velocity
%             hn = obj.m/(obj.rho*pi*obj.r^2)-obj.h0;
%             delta_h = hn;

            omega = 0.1; % Rotation speed, w [rad/s]
            theta = t*omega;
            delta_h = theta/(2*pi)*obj.lambda; % Cylinder height change due to motor rotation, [m]
            
            
            h = obj.h0 + delta_h;
            dx1dt = x2;
            dx2dt = 1/obj.m*(obj.m*obj.g - obj.mu*x2 - obj.rho*obj.g*pi*obj.r^2*h);
            dxdt = [dx1dt dx2dt]';
        end

        function sol = simulation(obj)
            sol = ode45(@(t,x) stateSpaceModel(obj,t,x),obj.tspan,obj.x0);
        end

        function plotResults(~, time, sol)
            subplot(1,2,1)
            plot(time, sol)
            xlabel('Time, t [sec]')
            legend('Depth','Velocity')
            subplot(1,2,2)
            plot(sol(1,:),sol(2,:))
            xlabel('Depth, d [m]')
            ylabel('Velocity, v [m/s]')
            axis equal
        end
    end
end














