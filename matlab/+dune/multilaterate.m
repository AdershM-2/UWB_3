function [pos, info] = multilaterate(anchorPos, ranges, opts)
%MULTILATERATE Robust weighted range multilateration (2D).
%   [pos, info] = dune.multilaterate(anchorPos, ranges, opts)
%
%   Levenberg-Marquardt with a pseudo-Huber robust loss (IRLS): marginal
%   outliers are smoothly down-weighted inside the solve instead of the old
%   hard MAD gate-and-resolve, matching the loss the MHE estimators use.
%   Cold starts are seeded by a closed-form squared-range linear solve
%   (SR-LS) rather than the anchor centroid, so the optimiser starts in the
%   right basin even with unfavourable geometry.
%
%   anchorPos [M x 3]  anchor antenna positions (m)
%   ranges    [M x 1]  measured ranges (m); NaN = anchor absent this sweep
%   opts.weights [M x 1] per-anchor weights (default all 1)
%   opts.tagZ    fixed tag antenna height (m, default 0.22 = anchor plane)
%   opts.x0      [1 x 2] initial guess (default: SR-LS closed-form seed)
%   opts.huberDelta  pseudo-Huber threshold (m, default 0.15 to match the
%                MHE); Inf = plain weighted least squares (calibration use)
%   opts.maxIter LM iterations (default 50)
%
%   pos  [1 x 2]  solved x,y (NaN if <3 usable ranges or diverged)
%   info.resid   [M x 1] signed residuals pred-meas (NaN where unused)
%   info.used    [M x 1] logical, anchors in the solve (all finite-range,
%                positive-weight anchors; robust down-weighting replaces
%                hard rejection - see wRobust for who got attenuated)
%   info.wRobust [M x 1] final IRLS robust weight in (0,1] (NaN where unused)
%   info.rmse    robust-weighted residual RMSE of the final solve
%   info.iters   LM iterations used
%   info.cov     [2 x 2] position covariance estimate

arguments
    anchorPos (:,3) double
    ranges (:,1) double
    opts.weights (:,1) double = ones(size(ranges))
    opts.tagZ (1,1) double = 0.22
    opts.x0 double = []
    opts.huberDelta (1,1) double = 0.15
    opts.maxIter (1,1) double = 50
end

M = numel(ranges);
sel = ~isnan(ranges) & opts.weights > 0;
info.resid = nan(M,1); info.used = false(M,1); info.wRobust = nan(M,1);
info.rmse = NaN; info.iters = 0; info.cov = nan(2);
pos = [NaN NaN];
if nnz(sel) < 3, return; end

% Physical-consistency screen: NO tag position can produce ranges violating
% the triangle inequality |d_i - d_j| <= ||a_i - a_j|| (+ slack). A range
% that violates it against a majority of the other answering anchors is
% garbage (stuck timestamp, wild reflection), not merely NLOS-inflated -
% excluded here so it cannot poison the squared-range seed, whose d^2 terms
% amplify gross outliers. NLOS-scale inflation passes and is left to the
% pseudo-Huber loss.
idx = find(sel);
if numel(idx) >= 3
    SLACK = 0.5;                          % m, well above NLOS inflation noise
    Ai = anchorPos(idx,:); di = ranges(idx);
    Dm = sqrt((Ai(:,1)-Ai(:,1)').^2 + (Ai(:,2)-Ai(:,2)').^2 + (Ai(:,3)-Ai(:,3)').^2);
    V  = abs(di - di') > Dm + SLACK | di + di' < Dm - SLACK;
    bad = sum(V, 2) > (numel(idx) - 1) / 2;
    sel(idx(bad)) = false;
end
if nnz(sel) < 3, return; end

% Degenerate-geometry gate: with (near-)collinear anchors the 2D solve has a
% mirror ambiguity and a singular normal matrix (e.g. only the three top-edge
% anchors answering). Require >=0.3 m of spread off the principal line.
sv = svd(anchorPos(sel,1:2) - mean(anchorPos(sel,1:2), 1));
if sv(2) < 0.3, return; end

x0 = opts.x0;
if isempty(x0) || any(~isfinite(x0))
    x0 = srlsSeed(anchorPos(sel,:), ranges(sel), opts.weights(sel), opts.tagZ);
end

[p, r, it, C, u] = lmSolve(anchorPos(sel,:), ranges(sel), opts.weights(sel), ...
                           opts.tagZ, x0, opts.maxIter, opts.huberDelta);

% Trim pass: the pseudo-Huber loss bounds a gross outlier's pull but never
% zeroes it, so an anchor whose CONVERGED robust weight has collapsed
% (|r| > ~3.9*delta) still biases the fix by centimetres. Drop it and
% re-solve once, warm-started - the old MAD gate's semantics, but judged
% from robust residuals instead of a contaminated LS fit. No-op for
% delta = Inf (u stays 1: calibration keeps every range).
TRIMW = 0.25;
if all(isfinite(p)) && any(u < TRIMW) && nnz(sel) - nnz(u < TRIMW) >= 3
    idx = find(sel);
    sel(idx(u < TRIMW)) = false;
    [p, r, it2, C, u] = lmSolve(anchorPos(sel,:), ranges(sel), ...
        opts.weights(sel), opts.tagZ, p, opts.maxIter, opts.huberDelta);
    it = it + it2;
end

if all(isfinite(p))
    pos = p;
    info.resid(sel)   = r;
    info.used         = sel;
    info.wRobust(sel) = u;
    v = opts.weights(sel).^2 .* u;      % combined weight actually solved with
    info.rmse  = sqrt(sum(v .* r.^2) / sum(v));
    info.iters = it;
    info.cov   = C;
end
end

function x0 = srlsSeed(A, d, w, tagZ)
% Closed-form squared-range seed: subtract the most-trusted anchor's range
% equation from the rest, leaving a linear system in [x y]. Exactly the
% right basin for LM; falls back to the weighted centroid if ill-posed.
d2 = max(d.^2 - (A(:,3) - tagZ).^2, 0);   % effective 2D squared ranges
[~, ref] = max(w);
i = true(numel(d), 1); i(ref) = false;
G = 2 * (A(i,1:2) - A(ref,1:2));
h = (sum(A(i,1:2).^2, 2) - sum(A(ref,1:2).^2)) - (d2(i) - d2(ref));
Gw = G .* w(i);
H = Gw' * G;                              % G' diag(w) G
if rcond(H) > 1e-9
    x0 = (H \ (Gw' * h))';
else
    x0 = sum(A(:,1:2) .* w, 1) / sum(w);
end
end

function [p, resid, iter, C, u] = lmSolve(A, d, w, tagZ, x0, maxIter, delta)
% IRLS-LM minimisation of sum_i w_i^2 * rho(r_i) over p = [x y], with
% rho the pseudo-Huber loss (rho(r) = r^2 for delta = Inf). Base weights
% enter squared, matching the previous sum (w_i r_i)^2 formulation.
dz2 = (A(:,3) - tagZ).^2;
wb = w.^2;
p = x0(:)';
lambda = 1e-3;
[resid, J] = residJac(p, A, d, dz2);
u = huberW(resid, delta);
cost = robustCost(resid, wb, delta);
iter = 0;
for iter = 1:maxIter
    v  = wb .* u;                      % IRLS: w^2 * psi(r)/r
    Jw = J .* v;
    H  = Jw' * J;                      % J' diag(v) J
    g  = Jw' * resid;                  % true robust gradient
    step = -(H + lambda * diag(diag(H))) \ g;
    pNew = p + step';
    [rNew, JNew] = residJac(pNew, A, d, dz2);
    cNew = robustCost(rNew, wb, delta);
    if cNew < cost
        p = pNew; resid = rNew; J = JNew; cost = cNew;
        u = huberW(resid, delta);      % re-weight on the accepted iterate
        lambda = max(lambda / 3, 1e-9);
        if norm(step) < 1e-6, break; end
    else
        lambda = lambda * 5;
        if lambda > 1e6, break; end
    end
end
% Covariance from the robust linearisation: sigma^2 * inv(J' V J)
dof = max(numel(d) - 2, 1);
v = wb .* u;
sigma2 = sum(v .* resid.^2) / dof;
Jw = J .* v;
Hf = Jw' * J;
Hf = (Hf + Hf') / 2;
if rcond(Hf) > 1e-12
    C = sigma2 * inv(Hf); %#ok<MINV>
else
    C = nan(2);
end
end

function u = huberW(r, delta)
% Pseudo-Huber IRLS weight psi(r)/r; ->1 for small residuals, ~delta/|r|
% for gross outliers (bounded influence). delta = Inf -> plain LS.
if isinf(delta)
    u = ones(size(r));
else
    u = 1 ./ sqrt(1 + (r ./ delta).^2);
end
end

function c = robustCost(r, wb, delta)
if isinf(delta)
    c = sum(wb .* r.^2);
else
    c = sum(wb .* delta^2 .* (sqrt(1 + (r ./ delta).^2) - 1));
end
end

function [r, J] = residJac(p, A, d, dz2)
dx = p(1) - A(:,1); dy = p(2) - A(:,2);
pred = sqrt(dx.^2 + dy.^2 + dz2);
pred = max(pred, 1e-6);
r = pred - d;
J = [dx ./ pred, dy ./ pred];
end
