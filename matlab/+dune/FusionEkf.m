classdef FusionEkf < handle
    %FUSIONEKF Loosely-coupled 2D UWB + IMU EKF (port of python fusion_ekf.py).
    %   State x = [px py vx vy bx by]':
    %     p - world position (m), v - world velocity (m/s),
    %     b - horizontal accelerometer bias in world frame (m/s^2).
    %
    %   Prediction: IMU-driven when world-frame linear acceleration is given
    %   (BNO085 quaternion-rotated), constant-velocity fallback otherwise.
    %   Updates: position fix (from dune.multilaterate, R from its covariance)
    %   and ZUPT (zero-velocity pseudo-measurement), both chi-squared gated.
    %
    %   ekf = dune.FusionEkf();
    %   ekf.predict(dt, aWorld);        % aWorld [] or [ax ay] m/s^2
    %   ekf.updatePosition(pos, R);     % pos [1x2], R [2x2]
    %   ekf.updateZupt();
    %   ekf.pos, ekf.vel, ekf.bias

    properties
        sigmaAccel   = 0.35   % m/s^2, accel white noise (IMU-driven predict)
        sigmaAccelCV = 0.8    % m/s^2, CV-fallback model uncertainty
        sigmaBiasRW  = 0.01   % m/s^2 / sqrt(s), accel-bias random walk
        posSigma     = 0.08   % m, fallback position-measurement sigma
        zuptSigma    = 0.05   % m/s, ZUPT pseudo-measurement sigma
        maxConsecReject = 10  % reinit from the fix after this many rejections
    end
    properties (SetAccess = private)
        x = zeros(6, 1)
        P = eye(6) * 1e4
        initialized = false
        lastNis = NaN
        lastAccepted = true
        nRejected = 0
        nReinit = 0
        consecReject = 0
    end

    methods
        function initialize(obj, pos)
            obj.x = [pos(1); pos(2); 0; 0; 0; 0];
            obj.P = diag([0.25, 0.25, 1.0, 1.0, 0.01, 0.01]);
            obj.initialized = true;
        end

        function p = pos(obj),  p = obj.x(1:2)'; end
        function v = vel(obj),  v = obj.x(3:4)'; end
        function b = bias(obj), b = obj.x(5:6)'; end

        function predict(obj, dt, aWorld)
            % predict(dt) or predict(dt, [])  -> CV fallback
            % predict(dt, [ax ay])            -> IMU-driven
            if ~obj.initialized || dt <= 0, return; end
            useImu = nargin >= 3 && numel(aWorld) == 2 && all(isfinite(aWorld));

            I2 = eye(2); Z2 = zeros(2);
            if useImu
                % px' = px + vx dt + 1/2 (a - b) dt^2 ; vx' = vx + (a - b) dt
                F = [I2, dt*I2, -0.5*dt^2*I2;
                     Z2, I2,    -dt*I2;
                     Z2, Z2,     I2];
                u = aWorld(:);
                B = [0.5*dt^2*I2; dt*I2; Z2];
                obj.x = F * obj.x + B * u;
                sa = obj.sigmaAccel;
            else
                F = [I2, dt*I2, Z2;
                     Z2, I2,    Z2;
                     Z2, Z2,    I2];
                obj.x = F * obj.x;
                sa = obj.sigmaAccelCV;
            end
            sb = obj.sigmaBiasRW;
            Q = [sa^2*(dt^3/3)*I2, sa^2*(dt^2/2)*I2, Z2;
                 sa^2*(dt^2/2)*I2, sa^2*dt*I2,       Z2;
                 Z2,               Z2,               sb^2*dt*I2];
            obj.P = F * obj.P * F' + Q;
        end

        function ok = update(obj, z, H, R)
            % Generic gated EKF update. Gate: chi2(dof, 0.95).
            CHI2_95 = [3.841, 5.991, 7.815];   % dof 1..3
            y = z(:) - H * obj.x;
            S = H * obj.P * H' + R;
            nis = y' * (S \ y);
            obj.lastNis = nis;
            ok = nis <= CHI2_95(min(numel(z), 3));
            obj.lastAccepted = ok;
            if ~ok
                obj.nRejected = obj.nRejected + 1;
                return;
            end
            K = (obj.P * H') / S;
            obj.x = obj.x + K * y;
            obj.P = (eye(6) - K * H) * obj.P;
            obj.P = (obj.P + obj.P') / 2;      % keep symmetric
        end

        function ok = updatePosition(obj, pos, R)
            if ~obj.initialized
                obj.initialize(pos);
                ok = true;
                return;
            end
            if nargin < 3 || isempty(R) || any(~isfinite(R(:)))
                R = eye(2) * obj.posSigma^2;
            end
            H = [eye(2), zeros(2, 4)];
            ok = obj.update(pos, H, R);
            % Divergence recovery: a healthy filter should not reject a long
            % streak of fixes. Re-anchor on the measurements when it does.
            if ok
                obj.consecReject = 0;
            else
                obj.consecReject = obj.consecReject + 1;
                if obj.consecReject >= obj.maxConsecReject
                    obj.initialize(pos);
                    obj.nReinit = obj.nReinit + 1;
                    obj.consecReject = 0;
                end
            end
        end

        function ok = updateZupt(obj)
            if ~obj.initialized, ok = false; return; end
            H = [zeros(2), eye(2), zeros(2)];
            ok = obj.update([0; 0], H, eye(2) * obj.zuptSigma^2);
        end
    end

    methods (Static)
        function aW = worldAccel(quat, acc)
            % Rotate BNO085 body-frame LINEAR acceleration (gravity-removed)
            % into the world frame; returns [ax ay] or [] if inputs invalid.
            % quat = [qw qx qy qz] (HostLink order).
            aW = [];
            if numel(quat) ~= 4 || numel(acc) ~= 3, return; end
            if any(~isfinite(quat)) || any(~isfinite(acc)), return; end
            n = norm(quat);
            if n < 0.5, return; end
            q = quat(:)' / n;
            w = q(1); xq = q(2); yq = q(3); zq = q(4);
            Rq = [1-2*(yq^2+zq^2),   2*(xq*yq - zq*w), 2*(xq*zq + yq*w);
                  2*(xq*yq + zq*w), 1-2*(xq^2+zq^2),   2*(yq*zq - xq*w);
                  2*(xq*zq - yq*w),  2*(yq*zq + xq*w), 1-2*(xq^2+yq^2)];
            a3 = Rq * acc(:);
            aW = a3(1:2)';
        end
    end
end
