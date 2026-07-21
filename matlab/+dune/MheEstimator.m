classdef MheEstimator < handle
    %MHEESTIMATOR Moving Horizon Estimator for 2D UWB positioning (CasADi/IPOPT).
    %   Experimental alternative to dune.FusionEkf. Instead of the EKF's
    %   recursive snap-to-latest-fix, the MHE keeps a sliding window of the
    %   last N sweeps and, each step, solves for the WHOLE [p,v] trajectory
    %   over the window that best fits:
    %     - every anchor range in every windowed sweep (robust pseudo-Huber,
    %       so outliers are smoothly down-weighted, not hard-gated),
    %     - a constant-velocity process model between steps,
    %     - an arrival cost tying the window start to the prior estimate
    %       (carries information forward as the window slides).
    %   Output is the state at the newest window step (a filter, comparable to
    %   the EKF). The window re-linearises the range geometry over N sweeps, so
    %   it handles the nonlinearity and answering-subset changes better than a
    %   single-point EKF linearisation.
    %
    %   Requires CasADi on the path (caller adds it):
    %     addpath('C:\Users\itisa\Downloads\casadi-3.7.0');
    %
    %   Usage:
    %     mhe = dune.MheEstimator(A.pos);      % A.pos = M x 3 anchor positions
    %     mhe.horizon = 10; mhe.build();
    %     [p, v, info] = mhe.push(rangeCorr, weights, dt, rawFixHint);
    %   push returns p = [NaN NaN] during warm-up (window not yet full); the
    %   caller falls back to the raw fix until then.

    properties
        horizon   = 10        % N sweeps in the window (~2 s at 5 Hz)
        sigmaR    = 0.05       % range measurement sigma (m)
        sigmaAccel = 0.8       % CV process accel white-noise (m/s^2)
        tagZ      = 0.24       % fixed tag antenna height (m)
        arrivalPos = 0.15      % arrival-cost sigma, position (m)
        arrivalVel = 0.6       % arrival-cost sigma, velocity (m/s)
        huberDelta = 0.15      % robust range-residual threshold (m)
        zuptSigma  = 0.03      % zero-velocity pseudo-meas sigma when still (m/s)
        maxJump    = 0.20      % m, jump-guard: reject output leaps beyond the
                               % CV prediction by more than this (IPOPT can land
                               % on a different near-optimal solution step-to-step)
        maxIter   = 60
    end
    properties (SetAccess = private)
        A                      % M x 3 anchor positions
        M
        solver                 % casadi nlpsol Function
        built = false
        buf = {}               % ring of struct(z Mx1, w Mx1, dt)
        Xw = []                % last window solution, 4N x 1
        prior = []             % arrival target [px py vx vy]'
        lastP = [NaN NaN]
        lastV = [0 0]
        nSolves = 0
        nJumps = 0
    end

    methods
        function obj = MheEstimator(anchorPos)
            obj.A = anchorPos;
            obj.M = size(anchorPos, 1);
        end

        function build(obj)
            % Construct the parametric NLP once; reused every push().
            import casadi.*
            N = obj.horizon; Ma = obj.M;
            Ax = obj.A(:,1); Ay = obj.A(:,2); dz = obj.A(:,3) - obj.tagZ;

            X     = MX.sym('X', 4, N);      % [px;py;vx;vy] per window step
            Z     = MX.sym('Z', Ma, N);     % measured ranges (0 where absent)
            W     = MX.sym('W', Ma, N);     % per-anchor weights (0 = absent)
            DT    = MX.sym('DT', N-1, 1);   % inter-step dt (s)
            STILL = MX.sym('STILL', N, 1);  % 1 where the tag was still (ZUPT)
            PRIOR = MX.sym('PRIOR', 4, 1);  % arrival target

            f = MX(0);
            d = obj.huberDelta; sR2 = obj.sigmaR^2; zs2 = obj.zuptSigma^2;
            for k = 1:N
                px = X(1,k); py = X(2,k);
                pred = sqrt((px - Ax).^2 + (py - Ay).^2 + dz.^2 + 1e-9);
                r = pred - Z(:,k);
                ph = d^2 * (sqrt(1 + (r ./ d).^2) - 1);   % pseudo-Huber (smooth)
                f = f + sum1(W(:,k) .* ph) / sR2;
                % Zero-velocity pseudo-measurement while still (parity with the
                % EKF's ZUPT; without it the window drifts on parked breathing).
                f = f + STILL(k) * sum1(X(3:4,k).^2) / zs2;
            end
            sigA = obj.sigmaAccel;
            for k = 1:N-1
                dt = DT(k);
                qp = 0.5 * sigA * dt^2 + 1e-6;
                qv = sigA * dt + 1e-6;
                dp = X(1:2,k+1) - X(1:2,k) - X(3:4,k) * dt;
                dv = X(3:4,k+1) - X(3:4,k);
                f = f + sum1(dp.^2) / qp^2 + sum1(dv.^2) / qv^2;
            end
            f = f + sum1((X(1:2,1) - PRIOR(1:2)).^2) / obj.arrivalPos^2 ...
                  + sum1((X(3:4,1) - PRIOR(3:4)).^2) / obj.arrivalVel^2;

            P = [Z(:); W(:); DT; STILL; PRIOR];
            nlp = struct('x', X(:), 'f', f, 'p', P);
            opts = struct('print_time', false, 'ipopt', ...
                struct('print_level', 0, 'sb', 'yes', 'max_iter', obj.maxIter, ...
                       'tol', 1e-5, 'acceptable_tol', 1e-4, 'mu_strategy', 'adaptive'));
            obj.solver = nlpsol('mhe', 'ipopt', nlp, opts);
            obj.built = true;
        end

        function reset(obj)
            obj.buf = {}; obj.Xw = []; obj.prior = [];
            obj.lastP = [NaN NaN]; obj.lastV = [0 0];
        end

        function [p, v, info] = push(obj, z, w, dt, pHint, still)
            % z,w: M x 1 corrected ranges / weights (NaN or w<=0 = absent).
            % dt:  seconds since the previous push. pHint: raw fix [1x2] to
            % seed the optimiser (optional; may be NaN). still: logical, tag
            % stationary this sweep -> zero-velocity pseudo-measurement.
            if nargin < 5, pHint = [NaN NaN]; end
            if nargin < 6, still = false; end
            z = z(:); w = w(:);
            bad = ~isfinite(z) | ~isfinite(w) | w <= 0;
            z(bad) = 0; w(bad) = 0;
            obj.buf{end+1} = struct('z', z, 'w', w, 'dt', max(dt, 1e-3), ...
                                    'still', double(logical(still)));

            N = obj.horizon;
            info = struct('warmup', true, 'cost', NaN, 'iters', 0);
            p = [NaN NaN]; v = [NaN NaN];
            if numel(obj.buf) < N, return; end          % warm-up: caller falls back
            if numel(obj.buf) > N, obj.buf(1) = []; end
            if ~obj.built, obj.build(); end

            % Assemble parameters over the window.
            Zm = zeros(obj.M, N); Wm = zeros(obj.M, N);
            DTv = zeros(N-1, 1); STv = zeros(N, 1);
            for k = 1:N
                Zm(:,k) = obj.buf{k}.z;
                Wm(:,k) = obj.buf{k}.w;
                STv(k)  = obj.buf{k}.still;
                if k >= 2, DTv(k-1) = obj.buf{k}.dt; end
            end

            % Warm start + arrival prior.
            if isempty(obj.Xw)
                seedP = pHint;
                if any(~isfinite(seedP)), seedP = mean(obj.A(:,1:2), 1); end
                X0 = repmat([seedP(:); 0; 0], N, 1);
                pr = [seedP(:); 0; 0];                  % first window ~ free fit
            else
                Xp = reshape(obj.Xw, 4, N);
                last = Xp(:,end) + [Xp(3:4,end) * DTv(end); 0; 0];
                X0 = reshape([Xp(:,2:end), last], [], 1);
                pr = obj.prior;
            end
            Pv = [Zm(:); Wm(:); DTv; STv; pr];

            r = obj.solver('x0', X0, 'p', Pv);
            X = full(r.x);
            obj.nSolves = obj.nSolves + 1;
            Xm = reshape(X, 4, N);
            p = Xm(1:2,end)'; v = Xm(3:4,end)';
            st = obj.solver.stats();
            solveOk = ~isfield(st, 'success') || st.success;

            info.warmup = false; info.cost = full(r.f); info.jumped = false;
            if isfield(st, 'iter_count'), info.iters = st.iter_count; end

            % Jump-guard: reject an output that leaps from its own constant-
            % velocity prediction by more than maxJump (or a non-converged
            % solve) -> fall back to the raw fix if sane, else the CV
            % prediction, and reseed the window so the bad solution does not
            % propagate through the arrival prior.
            if all(isfinite(obj.lastP)) && ...
               (~solveOk || norm(p - (obj.lastP + obj.lastV * dt)) > obj.maxJump)
                % Fall back to the raw fix (measurement-anchored, cannot run
                % away); only dead-reckon on the CV prediction if there is no
                % fix this sweep. Velocity is kept so the smoother continues.
                if all(isfinite(pHint))
                    p = pHint;
                else
                    p = obj.lastP + obj.lastV * dt;
                end
                v = obj.lastV;
                obj.reseedWindow(p, v, DTv);
                obj.nJumps = obj.nJumps + 1;
                info.jumped = true;
            else
                obj.Xw = X;
                obj.prior = Xm(:,2);                    % next window's arrival target
            end
            obj.lastP = p; obj.lastV = v;
        end

        function reseedWindow(obj, p, v, DTv)
            % Rebuild the window as a clean CV trajectory ending at (p,v) so a
            % rejected solve does not poison the next warm start / arrival prior.
            N = obj.horizon; dt = mean(DTv);
            Xm = zeros(4, N);
            for k = 1:N
                Xm(1:2,k) = p(:) - v(:) * (N - k) * dt;
                Xm(3:4,k) = v(:);
            end
            obj.Xw = Xm(:);
            obj.prior = Xm(:,2);
        end
    end
end
