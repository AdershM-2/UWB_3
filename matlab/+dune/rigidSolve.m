function out = rigidSolve(sHi, sLo, A, L, opts)
%RIGIDSOLVE Rigid-body joint solve of two tags a fixed distance L apart.
%   out = dune.rigidSolve(sHi, sLo, A, L)
%   out = dune.rigidSolve(sHi, sLo, A, L, rangeCorr=RC, tagZ=0.24, psi0=prevPsi)
%
%   The two tag antennas sit on a rigid plate a known distance L apart. Rather
%   than solve each tag independently and read yaw off the noisy pair, fit ALL
%   ranges from BOTH tags to a single rigid pose:
%       state x = [cx, cy, psi]                       (centre + heading)
%       p_hi = [cx, cy] + (L/2) [cos psi, sin psi]    (higher-id tag)
%       p_lo = [cx, cy] - (L/2) [cos psi, sin psi]    (lower-id  tag)
%   psi is the heading from the centre toward the HIGHER-id tag, matching
%   live_dual_tag's baseline yaw = atan2(tag_hi - tag_lo). L is enforced
%   EXACTLY (it is baked into the parameterisation), so the baseline can never
%   wander and both tags' anchors constrain centre + yaw jointly. This is the
%   step-7.3 rigid constraint (soft-penalty variant not needed: the hard
%   parameterisation is both simpler and exact) and the spatial core of the
%   rigid-body MHE (add [speed] + gyro->psi + no-slip for the temporal model).
%
%   sHi, sLo   parseRtlsLine sweep structs (.ids/.dist/.rx/.fp). Assign by id:
%              sHi = the larger tag id. Weights/corrections reuse solveSweep.
%   A          dune.loadAnchors struct (ids [Mx1], pos [Mx3])
%   L          inter-tag antenna spacing (m) — the hard constraint
%   opts.rangeCorr  dune.loadRangeCorrection struct or [] (per-anchor power bias)
%   opts.tagZ       tag antenna height (m, default 0.24)
%   opts.useGapWeights  NLOS soft weights (default true; via solveSweep)
%   opts.psi0       heading warm-start (rad); default from the independent pair
%   opts.maxIter    LM iterations (default 30)
%
%   out fields:
%     .ok        true if the solve converged with >=3 combined measurements
%     .c         [cx cy] centre (m)
%     .psi       heading (rad), toward the higher-id tag
%     .yawDeg    psi in degrees, wrapped to (-180,180]
%     .pHi,.pLo  [x y] each tag reconstructed from (c, psi, L)
%     .rmse      weighted range-residual RMSE (m)
%     .nUsed     combined measurements used
%     .iters     iterations run
%     .pHiInd,.pLoInd  independent single-tag solves (for comparison), or NaN

arguments
    sHi (1,1) struct
    sLo (1,1) struct
    A (1,1) struct
    L (1,1) double
    opts.rangeCorr = []
    opts.tagZ (1,1) double = 0.24
    opts.useGapWeights (1,1) logical = true
    opts.psi0 double = []
    opts.maxIter (1,1) double = 30
end

z = opts.tagZ;
half = L / 2;

% Reuse solveSweep for per-anchor corrected ranges + NLOS/MAD weights + an
% independent position (the joint init and the comparison baseline).
[pHi, iHi] = dune.solveSweep(sHi, A, rangeCorr=opts.rangeCorr, tagZ=z, ...
                             useGapWeights=opts.useGapWeights);
[pLo, iLo] = dune.solveSweep(sLo, A, rangeCorr=opts.rangeCorr, tagZ=z, ...
                             useGapWeights=opts.useGapWeights);

% Measurement rows: [anchor_xyz(1x3), d, w, sign] ; sign +1 hi, -1 lo.
[Ah, dh, wh] = pickRows(iHi, A);
[Al, dl, wl] = pickRows(iLo, A);
aXYZ = [Ah; Al];
dObs = [dh; dl];
wObs = [wh; wl];
sgn  = [ones(numel(dh),1); -ones(numel(dl),1)];

out = struct('ok', false, 'c', [NaN NaN], 'psi', NaN, 'yawDeg', NaN, ...
             'pHi', [NaN NaN], 'pLo', [NaN NaN], 'rmse', NaN, ...
             'nUsed', numel(dObs), 'iters', 0, ...
             'pHiInd', pHi, 'pLoInd', pLo);
if numel(dObs) < 3, return; end

% ---- initial guess ----
if all(isfinite(pHi)) && all(isfinite(pLo))
    c = (pHi + pLo) / 2;
    psi = atan2(pHi(2) - pLo(2), pHi(1) - pLo(1));
elseif all(isfinite(pHi))
    psi = 0; c = pHi - half * [cos(psi) sin(psi)];
elseif all(isfinite(pLo))
    psi = 0; c = pLo + half * [cos(psi) sin(psi)];
else
    c = mean(aXYZ(:,1:2), 1); psi = 0;      % anchor centroid fallback
end
if ~isempty(opts.psi0) && isfinite(opts.psi0), psi = opts.psi0; end

% ---- Levenberg-Marquardt on [cx cy psi] ----
sw = sqrt(max(wObs, 0));
lambda = 1e-3;
prevCost = inf;
iters = 0;
for it = 1:opts.maxIter
    iters = it;
    [r, J] = residJac(c, psi, half, z, aXYZ, dObs, sgn);
    rw = sw .* r;
    Jw = sw .* J;
    cost = sum(rw.^2);
    H = Jw.' * Jw;
    g = Jw.' * rw;
    % LM step with a couple of damping retries
    stepOk = false;
    for tryK = 1:6
        dlt = -(H + lambda * diag(diag(H) + 1e-9)) \ g;
        cN = c + dlt(1:2).'; psiN = psi + dlt(3);
        rN = residJac(cN, psiN, half, z, aXYZ, dObs, sgn);
        costN = sum((sw .* rN).^2);
        if costN < cost
            c = cN; psi = psiN; lambda = max(lambda / 3, 1e-6);
            stepOk = true;
            break;
        else
            lambda = min(lambda * 4, 1e6);
        end
    end
    if ~stepOk, break; end
    if abs(prevCost - costN) < 1e-9 * (1 + costN) || norm(dlt) < 1e-6
        prevCost = costN; break;
    end
    prevCost = costN;
end

rFinal = residJac(c, psi, half, z, aXYZ, dObs, sgn);
out.ok     = true;
out.c      = c;
out.psi    = psi;
out.yawDeg = mod(rad2deg(psi) + 180, 360) - 180;
out.pHi    = c + half * [cos(psi) sin(psi)];
out.pLo    = c - half * [cos(psi) sin(psi)];
out.rmse   = sqrt(mean((sw .* rFinal).^2) / max(mean(wObs), eps));
out.iters  = iters;
end

% ---------------------------------------------------------------------------
function [Axyz, d, w] = pickRows(info, A)
% Anchors this tag actually ranged this sweep, with corrected range + weight.
sel = isfinite(info.rangeCorr(:)) & isfinite(info.w(:)) & info.w(:) > 0 ...
      & ~info.rejected(:);
Axyz = A.pos(sel, :);
d    = info.rangeCorr(sel);
w    = info.w(sel);
d = d(:); w = w(:);
end

function [r, J] = residJac(c, psi, half, z, aXYZ, dObs, sgn)
% Residuals r = dist(tag, anchor) - dObs and Jacobian wrt [cx cy psi].
n = numel(dObs);
cx = c(1); cy = c(2);
cp = cos(psi); sp = sin(psi);
r = zeros(n, 1);
J = zeros(n, 3);
for k = 1:n
    px = cx + sgn(k) * half * cp;
    py = cy + sgn(k) * half * sp;
    dx = px - aXYZ(k,1);
    dy = py - aXYZ(k,2);
    dz = z  - aXYZ(k,3);
    rr = sqrt(dx*dx + dy*dy + dz*dz);
    r(k) = rr - dObs(k);
    if rr < 1e-9, rr = 1e-9; end
    ex = dx / rr; ey = dy / rr;          % d(dist)/d(px,py)
    % d(px,py)/dpsi = sgn*half*[-sin, cos]
    dpsi = ex * (sgn(k) * half * (-sp)) + ey * (sgn(k) * half * cp);
    J(k,:) = [ex, ey, dpsi];             % d(dist)/d[cx cy psi]
end
end
