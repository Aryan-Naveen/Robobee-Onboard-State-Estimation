classdef RobobeeEKF < handle
    %ROBOBEEEKF Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        H
        x
        P
        Q
        R
        dt
        config
    end
    
    methods
        function obj = RobobeeEKF(params, config)
            %ROBOBEEEKF Construct an instance of this class
            %   Detailed explanation goes here
              obj.H = [ 1 0 0 0 0 0 0 0 0 0;
                    0 1 0 0 0 0 0 0 0 0;
                    0 0 1 0 0 0 0 0 0 0;
                    0 0 0 0 0 0 1 0 0 0];
              obj.Q = diag(params(1:10));
              obj.R = diag(params(11:14)); 
              obj.x = [0 0 0 0 0 0 0 0 0 0]';  

                % state covariance
              obj.P = 10*eye(10);

              obj.dt = config.sampling_time;
              obj.config = config;
              obj.config.bw = 2e-4;
              obj.config.rw = 9e-3;
        end
        
        function x = update(obj, z, u)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            A = Ajacob(obj.x, u, obj.config, obj.dt);
            xp = fx(obj.x, u, obj.config, obj.dt);
            Pp = A*obj.P*A' + obj.Q;
            
            K = Pp*obj.H'*pinv(obj.H*Pp*obj.H' + obj.R);
            obj.x = xp + K*(z - obj.H*xp);
            obj.P = Pp - K*obj.H*Pp;
            x = [obj.x(1); obj.x(2); obj.x(3); obj.x(7)];
            
        end
    end
end


function xp = fx(xhat, U, config, dt)
%
%
phi   = xhat(1);
theta = xhat(2);
psi = xhat(3);

p = xhat(4);
q = xhat(5);
r = xhat(6);

z = xhat(7);

vx = xhat(8);
vy = xhat(9);
vz = xhat(10);

F = U(4);
a = (1/config.m)*F;
Tx = U(1);
Ty = U(2);
Tz = U(3);

xdot = zeros(3, 1);
xdot(1) = p + q*sin(phi)*tan(theta) + r*cos(phi)*tan(theta);
xdot(2) =     q*cos(phi)            - r*sin(phi);
xdot(3) =     q*sin(phi)*sec(theta) + r*cos(phi)*sec(theta);
xdot(4) = (-(1/config.Ixx)*config.bw*config.rw^2)*p + ((1/config.Ixx)*config.bw*config.rw)*vy + (1/config.Ixx)*Tx;
xdot(5) = (-(1/config.Iyy)*config.bw*config.rw^2)*q + ((1/config.Iyy)*config.bw*config.rw)*vx + (1/config.Iyy)*Ty;
xdot(6) = (-(1/config.Izz)*config.bw*config.rw^2)*r + (1/config.Izz)*Tz;
xdot(7) = vz;
xdot(8) = (a-config.g)*theta-((1/config.m)*config.bw)*vx;
xdot(9) = (a-config.g)*phi-((1/config.m)*config.bw)*vy;
xdot(10) = -1.2*vz + a - config.g;



xp = xhat + xdot*dt;

end
%------------------------------
function A = Ajacob(xhat, U, config, dt)
%
% xhat = [phi theta psi wx wy wz z vx vy vz P_]
A = zeros(10, 10);


phi   = xhat(1);
theta = xhat(2);
psi = xhat(3);

p = xhat(4);
q = xhat(5);
r = xhat(6);

z = xhat(7);

vx = xhat(8);
vy = xhat(9);
vz = xhat(10);

A = zeros(10, 10);


% roll
A(1,1) = q*cos(phi)*tan(theta)   - r*sin(phi)*tan(theta);
A(1,2) = q*sin(phi)*sec(theta)^2 + r*cos(phi)*sec(theta)^2;
A(1,4) = 1;
A(1,5) = sin(phi)*tan(theta);
A(1,6) = cos(phi)*tan(theta);

% pitch
A(2,1) = -q*sin(phi) - r*cos(phi);
A(2,5) = cos(phi);
A(2,6) = -sin(phi);


%yaw 
A(3,1) = q*cos(phi)*sec(theta)            - r*sin(phi)*sec(theta);
A(3,2) = q*sin(phi)*sec(theta)*tan(theta) + r*cos(phi)*sec(theta)*tan(theta);
A(3,5) = sin(phi)*sec(theta);
A(3,6) = cos(phi)*sec(theta);

% wx
A(4,4) = -(1/config.Ixx)*config.bw*config.rw^2;
A(4,9) = (1/config.Ixx)*config.bw*config.rw;

% wy
A(5,5) = -(1/config.Iyy)*config.bw*config.rw^2;
A(5,8) = (1/config.Iyy)*config.bw*config.rw;

%wz
A(6,6) = -(1/config.Izz)*config.bw*config.rw^2;

%z 
A(7, 10) = 1;

% vx
A(8, 2) = -config.g;
A(8, 5) = (1/config.m)*config.bw*config.rw;
A(8, 8) = -(1/config.m)*config.bw;

% vy
A(9, 1) = -config.g;
A(9, 4) = (1/config.m)*config.bw*config.rw;
A(9, 9) = -(1/config.m)*config.bw;

% vz
A(10, 10) = -(1/config.m)*config.bw;


A = eye(10) + A*dt;
end

