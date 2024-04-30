classdef ViconData
   properties
    config = [];
    pos = [];
    vel = [];
    acc = [];
    quat = [];
    omega = [];
    theta = [];
    time = [];

    %Commanded Inputs
    thrust = [];
    torques = [];
   end
   methods
      function obj = ViconData(yout, config)
         % COM Vicon Tracking 
         obj.pos = [yout(:,44) yout(:,45) yout(:,46)];
         obj.vel = [yout(:,41) yout(:,42) yout(:,43)];
         obj.acc = [filter([1 -1],[1],obj.vel(:, 1))*config.sampling_f filter([1 -1],[1],obj.vel(:, 2))*config.sampling_f filter([1 -1],[1],obj.vel(:, 3))*config.sampling_f];
         obj.quat = [yout(:,18) yout(:,19) yout(:,20) yout(:,21)];
         obj.omega = [yout(:,15) yout(:,16) yout(:,17)];
         obj.theta = [yout(:, 9) yout(:, 10) yout(:, 11)];

         obj.time = yout(:,1);          
         obj.config = config;
        
         % Commanded inputs
         obj.thrust = yout(:, 57);
         obj.torques = [yout(:,55), yout(:,56), yout(:,64)];
      end

      function acc = getAcceleration(obj, t)
          acc = obj.acc(t, :);
      end

     function acc = getVelocity(obj, t)
          acc = obj.vel(t, :);
      end

      function motion = getMotionData(obj, t)
         motion = [obj.pos(t, :) obj.vel(t, :) obj.acc(t, :) obj.quat(t, :) obj.omega(t, :)];
      end
      function omega = getOmegaAtTimet(obj, t)
          omega = obj.omega(t, :);
      end
      function start_delay_time_index = getStart(obj)
         I = find(obj.time-obj.config.start_delay<0);
         start_delay_time_index = I(end);
      end
      function theta = getThetas(obj)
          theta = obj.theta(obj.getStart():obj.getEnd(), :);
      end
    function theta = getThetasAtTimet(obj, t)
          theta = obj.theta(t, :);
      end
      function theta_ = getThetaVals(obj, t)
          theta_ = obj.theta(t, 2:3);
      end
      function end_time_index = getEnd(obj)
         I = find(obj.time-obj.config.running_time<0);
         end_time_index = I(end);
      end
      function time = getFlightTime(obj)
          time = obj.time(obj.getStart():obj.getEnd());
      end
      function t = getTime(obj, k)
          t = obj.time(k);
      end
      function plotPosition(obj)
          t0 = obj.getStart();
          tf = obj.getEnd();
          plot(obj.time(t0:tf), obj.pos(t0:tf, :))
          legend({'X','Y','Z'});
      end
      function plotControlInput(obj)
          t0 = obj.getStart();
          tf = obj.getEnd();
          u = [obj.thrust(t0:tf)*obj.config.scale, obj.torques(t0:tf, :)*obj.config.scale^2];
          plot(obj.time(t0:tf), u)
          legend({'Ft','roll torque','pitch torque', 'yaw torque'});
      end
      function ft = getThrust(obj, t)
          ft = obj.thrust(t);
      end
      function z = getZ(obj, t)
          z = obj.pos(t, 3);
      end
      function torque = getTorques(obj, t)
          torque = obj.torques(t, :);
      end
      function u = getDesiredControlInput(obj, t)
          u = [obj.torques(t, :), obj.getThrust(t)];
      end
      function q0 = getInitState(obj)
          t0 = obj.getStart();
          q0 = [obj.theta(t0, 1); obj.omega(t0, 1); obj.vel(t0, 2)];
      end
   end
end