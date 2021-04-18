

classdef QuadrotorClass
    
    properties (Constant)
        % CONSTANTS
        g = 9.81;   % The gravitational acceleration [m/s^2]
        l =  0.2;   % Distance from the center of mass to each rotor [m]
        m =  0.5;   % Total mass of the quadrotor [kg]
        I = [1.24, 1.24, 2.48]; % Mass moment of inertia [kg m^2]
        mu = 3.0;   % Maximum thrust of each rotor [N]
        sigma = 0.01; % The proportionality constant relating thrust to torque [m]
    end
    
    properties (SetAccess = private)
        p;
    end
    
    properties
        % Member Variables
        z0;% initial conditions
        r;
        n;
        u;
    end
    
    methods
        % Constructor
        function obj = QuadrotorClass(initialConditions, extForces, moment, input)
            obj.z0 = initialConditions; % 12x1 vertical vector 
            obj.r = extForces;  % 3x1 vertical vector
            obj.n = moment; % 3x1 vertical vector
            obj.u = input;  % 4x1 vertical vector
            
            obj.p = [obj.g obj.l obj.m obj.I obj.mu obj.sigma]; % parametric vector
        end
        
        % Graph of x, xdot, alpha, and omega
        function plotResults(this, t, z)
            yaxis_labels = {'x', '$\alpha$', '$\dot{x}$', '$\omega$'};
            
            for i=1:4
                ax(i) = subplot(2,2,i,'NextPlot','Add','Box','on','XGrid','on','YGrid','on',...
                            'Xlim',[t(1), t(end)],...
                            'TickLabelInterpreter','LaTeX','FontSize',14);
                xlabel('t','Interpreter','LaTeX','FontSize',14);
                ylabel(yaxis_labels(1,i), 'Interpreter', 'LaTeX', 'FontSize', 14);
            end

            plot(ax(1), t, z(:,1:3), 'LineWidth', 1.5);
            legend(ax(1), {'$x_1$', '$x_2$', '$x_3$'},... 
                'Interpreter', 'LaTeX', 'FontSize', 9, 'Location', 'best');
            title(ax(1), '$\bf Position / Time$','Interpreter','LaTeX','FontSize',10);
%             xlabel(ax(1), 't','Interpreter','LaTeX','FontSize',14);

            plot(ax(3), t, z(:,4:6), 'LineWidth', 1.5);
            legend(ax(3), {'$\phi$', '$\theta$', '$\psi$'},...
                'Interpreter', 'LaTeX', 'FontSize', 9, 'Location', 'best');
            title(ax(3), '$\bf Rotation / Time$','Interpreter','LaTeX','FontSize',10);

            plot(ax(2), t, z(:,7:9), 'LineWidth', 1.5);
            legend(ax(2), {'$\dot{x}_1$', '$\dot{x}_2$', '$\dot{x}_3$'},...
                'Interpreter', 'LaTeX', 'FontSize', 9, 'Location', 'best');
            title(ax(2), '$\bf Velocity / Time$','Interpreter','LaTeX','FontSize',10);

            plot(ax(4), t, z(:,10:12), 'LineWidth', 1.5);
            legend(ax(4), {'$\omega_1$', '$\omega_2$', '$\omega_3$'},...
                'Interpreter', 'LaTeX', 'FontSize', 9, 'Location', 'best');
            title(ax(4), '$\bf Angular Velocity / Time$','Interpreter','LaTeX','FontSize',10);
        end
        
%         function pos = silhouettePositions(z)
%             
%         end
                
        function rotorPos, bodyPos = quadrotorPositions(z,N,loc)
            for i=1:4

                ctr(i,:) = z(1:3) + loc(i,:)*R';
                rotorPos(i) = ones(N,1)*z(1:3) + (ones(N,1)*loc(i,:) + circle)*R'; 

            end
            bodyPos = [ctr([1 3],1), NaN, ctr([2 4],1);
                       ctr([1 3],2), NaN, ctr([2 4],2);
                       ctr([1 3],3), NaN, ctr([2 4],3)];
        end
        
        function R = quadrotorRotation(this, a1, a2, a3)
            R = [ cos(a2)*cos(a3), sin(a1)*sin(a2)*cos(a3) - cos(a1)*sin(a3), sin(a1)*sin(a3) + cos(a1)*sin(a2)*cos(a3);
                  cos(a2)*sin(a3), cos(a1)*cos(a3) + sin(a1)*sin(a2)*sin(a3), cos(a1)*sin(a2)*sin(a3) - sin(a1)*cos(a3);
                  -sin(a2),        sin(a1)*cos(a2),                           cos(a1)*cos(a2)];
        end
            
    end
    
end