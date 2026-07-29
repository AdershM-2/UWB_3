function out = rover_run_report_ekf(matFile, opts)
%ROVER_RUN_REPORT_EKF Score a rover_teleop_uwb capture with dune.FusionEkf
%   INSTEAD OF the rigid two-tag MHE, for direct comparison against
%   rover_run_report's numbers on the same run.
%
%   out = rover_run_report_ekf("...\rover_uwb_x.mat")
%
%   The rigid MHE fuses both tags jointly (shared heading/baseline model).
%   FusionEkf has no such coupling, so here each tag runs its OWN independent
%   position-only EKF (IMU-driven prediction on the rear tag, which carries
%   the BNO085 tail; CV prediction on the front tag) over that tag's own
%   sweeps. The two filtered tracks are then averaged at each timestamp to
%   get a centre estimate - valid because the tags are mounted symmetrically
%   about the rig centre (dune.MheRigid's own geometry assumption), but
%   WITHOUT the rigid model's heading coupling or forward-only constraint.
%   Everything downstream (truth alignment, RMSE, plot) matches
%   rover_run_report exactly so the two reports are comparable apples-to-apples.

arguments
    matFile string = ""
    opts.tagZ (1,1) double = 0.24
    opts.plot (1,1) logical = true
    opts.trimStartS (1,1) double = 0   % s dropped from the START of the truth-
                                       % alignment window (pickup/settle transient)
    opts.trimEndS (1,1) double = 0     % s dropped from the END (set-down)
end

if matFile == ""
    d = dir(fullfile(dune.rootDir(), 'results', 'rover_runs', 'rover_uwb_*.mat'));
    assert(~isempty(d), 'no rover_uwb_*.mat found');
    [~, i] = max([d.datenum]);
    matFile = fullfile(d(i).folder, d(i).name);
end
fprintf('Loading %s\n', matFile);
S = load(matFile);
data = S.data; meta = S.metadata; uwb = data.uwb;
A = dune.loadAnchors();
RC = dune.loadRangeCorrection();

offBody = meta.uwb.apriltag_offset_body(:)';
frontTag = meta.uwb.frontTag; rearTag = meta.uwb.rearTag;
nS = numel(uwb.t_posix);
fprintf('%d UWB sweeps | %d rover samples | AprilTag offset [%.4f %.4f]\n', ...
        nS, numel(data.time), offBody);

%% ---- per-sweep raw fix (unchanged reference, same as rover_run_report) --
Praw = nan(nS, 2);
prev = [];
for i = 1:nS
    have = find(~isnan(uwb.R(i, :)));
    if numel(have) < 3, continue; end
    sw = struct('tag', uwb.tag(i), 'ids', uwb.anchorIds(have), ...
                'dist', uwb.R(i, have), 'rx', uwb.RX(i, have), 'fp', uwb.FP(i, have));
    p = dune.solveSweep(sw, A, rangeCorr=RC, tagZ=opts.tagZ, x0=prev);
    Praw(i, :) = p;
    if all(isfinite(p)), prev = p; end
end
fprintf('raw per-sweep fixes: %d/%d solved (%.0f%%)\n', ...
        sum(~isnan(Praw(:,1))), nS, 100*mean(~isnan(Praw(:,1))));

%% ---- independent per-tag EKF ---------------------------------------------
[Pf, tf] = runTagEkf(uwb, A, RC, frontTag, opts.tagZ);
[Pr, tr_] = runTagEkf(uwb, A, RC, rearTag,  opts.tagZ);
fprintf('EKF: front %d/%d poses, rear %d/%d poses\n', ...
        sum(~isnan(Pf(:,1))), numel(tf), sum(~isnan(Pr(:,1))), numel(tr_));

% combine onto the full sweep time grid: hold each tag's last filtered fix
Cf = holdInterp(tf, Pf, uwb.t_posix);
Cr = holdInterp(tr_, Pr, uwb.t_posix);
Pose = nan(nS, 2);
both = all(isfinite(Cf), 2) & all(isfinite(Cr), 2);
Pose(both, :) = (Cf(both, :) + Cr(both, :)) / 2;
onlyF = all(isfinite(Cf),2) & ~both; Pose(onlyF,:) = Cf(onlyF,:);
onlyR = all(isfinite(Cr),2) & ~both & ~onlyF; Pose(onlyR,:) = Cr(onlyR,:);
fprintf('EKF centre (front+rear averaged): %d/%d sweeps (%.0f%%)\n', ...
        sum(~isnan(Pose(:,1))), nS, 100*mean(~isnan(Pose(:,1))));

%% ---- AprilTag truth, lever-arm corrected (identical to rover_run_report) -
okT = ~isnan(data.apriltag_pos(:,1));
tT = data.t_posix(okT);
xyT = data.apriltag_pos(okT, 1:2);
yawT = data.apriltag_euler(okT, 3);
cT = xyT - [cos(yawT).*offBody(1) - sin(yawT).*offBody(2), ...
            sin(yawT).*offBody(1) + cos(yawT).*offBody(2)];
fprintf('AprilTag truth samples: %d (%.0f%% of rover samples)\n', ...
        sum(okT), 100*mean(okT));

%% ---- trim start/end pickup transients from SCORING only -----------------
if opts.trimStartS > 0 || opts.trimEndS > 0
    tKeep = tT >= (tT(1) + opts.trimStartS) & tT <= (tT(end) - opts.trimEndS);
    fprintf('trim: dropping %.1fs start / %.1fs end -> %d/%d truth samples kept\n', ...
            opts.trimStartS, opts.trimEndS, sum(tKeep), numel(tT));
    tT = tT(tKeep); cT = cT(tKeep, :);
end

%% ---- align to truth, fit 2D rigid transform ------------------------------
out = struct('file', matFile, 'Praw', Praw, 'Pose', Pose, 'cT', cT, 'tT', tT);
src = Pose; srcT = uwb.t_posix;
good = ~isnan(src(:,1));
if sum(good) > 20 && numel(tT) > 20
    Ei = interp1(srcT(good), src(good,:), tT, 'linear', NaN);
    m = all(isfinite(Ei),2) & all(isfinite(cT),2);
    fprintf('overlapping samples for alignment: %d\n', sum(m));
    if sum(m) > 20
        [Rr, ttr, rms] = fitRigid2D(Ei(m,:), cT(m,:));
        Ea = (Rr * Ei(m,:)')' + ttr;
        err = vecnorm(Ea - cT(m,:), 2, 2);
        fprintf('\n=== EKF vs AprilTag truth (after 2D rigid alignment) ===\n');
        fprintf('  rotation %.1f deg, translation [%.2f %.2f] m\n', ...
                rad2deg(atan2(Rr(2,1),Rr(1,1))), ttr);
        fprintf('  RMSE %.0f mm | median %.0f mm | p95 %.0f mm | max %.0f mm\n', ...
                1000*rms, 1000*median(err), 1000*prctile(err,95), 1000*max(err));
        out.PoseAligned = nan(size(src));
        out.PoseAligned(good,:) = (Rr * src(good,:)')' + ttr;
        pathU = sum(vecnorm(diff(Ea), 2, 2));
        pathT = sum(vecnorm(diff(cT(m,:)), 2, 2));
        fprintf('  path length: EKF %.2f m vs truth %.2f m (%.2fx -> jitter)\n', ...
                pathU, pathT, pathU / max(pathT, 1e-6));
        out.err = err; out.Ealigned = Ea; out.cTm = cT(m,:); out.tm = tT(m);
        out.R = Rr; out.t = ttr; out.pathRatio = pathU / max(pathT, 1e-6);
    end
end

%% ---- plot -----------------------------------------------------------------
if ~opts.plot, return; end
TRAJ_LIM = [-3 3 -2 2];
fig = figure('Position',[30 30 1100 420]);
tiledlayout(1,3,'TileSpacing','compact','Padding','compact');

nexttile; hold on; grid on; axis equal;
plot(A.pos(:,1), A.pos(:,2), 'k^','MarkerFaceColor','y','MarkerSize',9);
text(A.pos(:,1)+0.05, A.pos(:,2), compose('A%d', A.ids), 'Clipping','on');
plot(Praw(:,1), Praw(:,2), '.', 'Color',[.75 .75 .75], 'MarkerSize',4);
if any(~isnan(Pose(:,1)))
    plot(Pose(:,1), Pose(:,2), '-', 'Color',[.2 .4 .85], 'LineWidth',1.2);
end
title('UWB frame: raw fixes (grey) + EKF centre (blue)'); xlabel('x (m)'); ylabel('y (m)');
lockTraj(TRAJ_LIM);

nexttile; hold on; grid on; axis equal;
plot(cT(:,1), cT(:,2), 'b.-', 'MarkerSize',5);
title(sprintf('AprilTag truth (Kinect frame), %d samples', size(cT,1)));
xlabel('x (m)'); ylabel('y (m)'); lockTraj(TRAJ_LIM);

nexttile; hold on; grid on; axis equal;
if isfield(out,'Ealigned')
    plot(out.PoseAligned(:,1), out.PoseAligned(:,2), '-', ...
         'Color',[.65 .75 1], 'LineWidth',0.8);
    plot(out.cTm(:,1), out.cTm(:,2), 'b-', 'LineWidth',1.4);
    plot(out.Ealigned(:,1), out.Ealigned(:,2), '-', 'Color',[.1 .2 .8], 'LineWidth',1.1);
    legend({'EKF full rate','truth','EKF @ truth times'}, 'Location','best');
    title(sprintf('aligned: RMSE %.0f mm, path %.1fx truth', ...
                  1000*sqrt(mean(out.err.^2)), out.pathRatio));
else
    title('not enough overlap to align');
end
xlabel('x (m)'); ylabel('y (m)'); lockTraj(TRAJ_LIM);

f = replace(char(matFile), '.mat', '_report_ekf.png');
saveas(fig, f);
fprintf('\nfigure: %s\n', f);
out.figFile = f;
end

%% ── helpers ─────────────────────────────────────────────────────────────
function [Pekf, thost] = runTagEkf(uwb, A, RC, tagId, tagZ)
% Independent single-tag FusionEkf over just this tag's own sweeps, in its
% own time order - identical chain to ekf_replay.m (solveSweep -> predict ->
% gated position update -> ZUPT on IMU stillness). IMU is only present on
% the rear (240) tag; the front tag falls back to plain CV prediction.
idx = find(uwb.tag == tagId);
thost = uwb.t_posix(idx);
n = numel(idx);
Pekf = nan(n, 2);
if n == 0, return; end
ekf = dune.FusionEkf(); ekf.robust = true;
prevRaw = []; tPrev = NaN;
histA = nan(1,8); histG = nan(1,8);
for k = 1:n
    i = idx(k);
    dt = 0; if isfinite(tPrev), dt = min(max(thost(k) - tPrev, 0), 1); end
    tPrev = thost(k);

    aW = []; still = false;
    if all(isfinite(uwb.quat(i,:)))
        histA = [histA(2:end), norm(uwb.acc(i,:))];
        histG = [histG(2:end), norm(uwb.gyro(i,:))];
        still = all(isfinite(histA)) && max(histA) < 0.12 && max(histG) < 0.05;
        aW = dune.FusionEkf.worldAccel(uwb.quat(i,:), uwb.acc(i,:));
    end
    ekf.stillMode = still;
    ekf.predict(dt, aW);

    have = find(~isnan(uwb.R(i,:)));
    if numel(have) >= 3
        sw = struct('tag', uwb.tag(i), 'ids', uwb.anchorIds(have), ...
                    'dist', uwb.R(i,have), 'rx', uwb.RX(i,have), 'fp', uwb.FP(i,have));
        [p, info] = dune.solveSweep(sw, A, rangeCorr=RC, tagZ=tagZ, x0=prevRaw);
        if all(isfinite(p))
            prevRaw = p;
            ekf.updatePosition(p, adaptiveR(info, ekf.posSigma));
        end
    end
    if still && ekf.initialized, ekf.updateZupt(); end
    if ekf.initialized, Pekf(k,:) = ekf.pos; end
end
end

function R = adaptiveR(info, posSigma)
% Port of the old runtime's _adaptive_R (same as ekf_replay.m): LM covariance
% inflated by solver RMSE, mean NLOS gap, and a DOP proxy (clipped 1..50x).
if all(isfinite(info.cov(:)))
    Rb = info.cov + eye(2) * 0.02^2;
else
    Rb = eye(2) * posSigma^2;
end
rmsF = 1 + (info.rmse / 0.05)^2;
g = info.gap(isfinite(info.gap) & isfinite(info.range));
if isempty(g), nlosF = 1; else, nlosF = 1 + mean(max(0, g / 6)); end
dop = sqrt(trace(Rb));
dopF = 1 + max(0, (dop - 0.05) / 0.05);
R = Rb * min(max(rmsF * nlosF * dopF, 1), 50);
end

function Y = holdInterp(t, X, tq)
% Zero-order hold of X(t) at query times tq (NaN before the first sample).
Y = nan(numel(tq), size(X,2));
if numel(t) < 2, return; end
for c = 1:size(X,2)
    Y(:,c) = interp1(t, X(:,c), tq, 'previous');
end
end

function lockTraj(lim)
axis(lim); daspect([1 1 1]); pbaspect([diff(lim(1:2)) diff(lim(3:4)) 1]);
end

function [R, t, rms] = fitRigid2D(P, Q)
mp = mean(P,1); mq = mean(Q,1);
Pc = P - mp; Qc = Q - mq;
H = Pc' * Qc;
[U,~,V] = svd(H);
D = diag([1, sign(det(V*U'))]);
R = V * D * U';
t = mq - (R*mp')';
E = (R*P')' + t - Q;
rms = sqrt(mean(sum(E.^2,2)));
end
