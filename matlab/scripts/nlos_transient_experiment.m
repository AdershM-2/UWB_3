function out = nlos_transient_experiment(logFile, opts)
%NLOS_TRANSIENT_EXPERIMENT Demonstrate the NLOS gap-weighting on a REAL
%   transient blocking event, replaying ONE capture twice (weights on/off).
%
%   out = nlos_transient_experiment("matlab\logs\rtls_log_x.jsonl")
%   out = nlos_transient_experiment(log, truthXY=[x y])   % if the tag was
%                                                          % parked at a known spot
%
%   WHY THIS EXPERIMENT: the project's own static NLOS ablation (19 parked,
%   UNOBSTRUCTED dwells) showed weights-on vs off was a wash - expected,
%   because it never exercised a blocked path. The mechanism (gapWeights.m)
%   exists for TRANSIENT blocking (a person or the rover's own body stepping
%   into a link). This script replays a capture that included real blocking
%   and shows the three things the report describes:
%     1. the physical signature: rx-fp gap jumps when a path is blocked
%     2. the range bias: that anchor's raw range reads LONG while blocked
%     3. the protection: position error/spread with weights ON vs OFF,
%        specifically DURING the auto-detected blocked windows
%
%   Blocked windows are auto-detected from the gap itself (gap > blockGapDb
%   sustained for > blockMinS) - no manual timestamping needed. Requires the
%   tag to be roughly STATIONARY during the capture (parked-block protocol);
%   this isolates the NLOS effect from motion.
%
%   opts.truthXY: if you know the parked position (m), position error is
%   reported directly; otherwise spread-around-median is used instead.

arguments
    logFile (1,1) string
    opts.tagId (1,1) double = 240
    opts.tagZ (1,1) double = 0.24
    opts.truthXY double = []          % [x y], optional
    opts.blockRelDb (1,1) double = 4  % "blocked" = gap this far ABOVE that
                                      % anchor's OWN clear-time baseline, not
                                      % an absolute cutoff - some anchors
                                      % (e.g. a chronically part-obstructed
                                      % one) sit at an elevated gap all the
                                      % time, and an absolute threshold close
                                      % to that baseline mistakes their normal
                                      % noise for a deliberate block
    opts.blockMinS (1,1) double = 2   % must sustain this long to count
    opts.blockCloseS (1,1) double = 1 % bridge dropouts shorter than this
                                      % (real blocking is rarely a clean
                                      % constant gap - a hand shifting mid-
                                      % block dips below threshold briefly;
                                      % close small gaps before the min-run
                                      % filter so those don't fragment away)
end

logFile = resolveLogPath(logFile);

A = dune.loadAnchors();
RC = dune.loadRangeCorrection();
S = dune.readSessionLog(logFile, A.ids);
T = S.tab(S.tab.tag == opts.tagId, :);
assert(height(T) > 20, 'not enough sweeps for tag %d in %s', opts.tagId, logFile);
t = T.thost - T.thost(1);
if all(isnan(t)), t = (T.tms - T.tms(1)) / 1000; end
nA = numel(A.ids);

%% ---- replay every sweep TWICE: gap-weighted vs unweighted -----------------
Pw = nan(height(T), 2); Pu = nan(height(T), 2);
gap = nan(height(T), nA); rangeRaw = nan(height(T), nA); wUsed = nan(height(T), nA);
prevW = []; prevU = [];
for i = 1:height(T)
    have = find(~isnan(T.R(i,:)));
    if numel(have) < 3, continue; end
    sw = struct('tag', opts.tagId, 'ids', A.ids(have), 'dist', T.R(i,have), ...
                'rx', T.RX(i,have), 'fp', T.FP(i,have));
    [pw, infoW] = dune.solveSweep(sw, A, rangeCorr=RC, tagZ=opts.tagZ, x0=prevW, ...
                                  useGapWeights=true);
    [pu, ~]     = dune.solveSweep(sw, A, rangeCorr=RC, tagZ=opts.tagZ, x0=prevU, ...
                                  useGapWeights=false);
    Pw(i,:) = pw; if all(isfinite(pw)), prevW = pw; end
    Pu(i,:) = pu; if all(isfinite(pu)), prevU = pu; end
    gap(i,:) = infoW.gap'; rangeRaw(i,:) = infoW.range'; wUsed(i,:) = infoW.w';
end

%% ---- auto-detect blocked windows per anchor from the gap ------------------
% Threshold is RELATIVE to each anchor's own clear-time baseline (20th
% percentile, robust to a block being a minority of samples) - an absolute
% cutoff mistakes a chronically part-obstructed anchor's normal noise for a
% deliberate block (that anchor never has a low "clear" gap to compare to).
dt = median(diff(t), 'omitnan');
minSamples = max(1, round(opts.blockMinS / max(dt, 1e-3)));
closeSamples = max(0, round(opts.blockCloseS / max(dt, 1e-3)));
baseline = nan(1, nA);
blocked = false(size(gap));
for a = 1:nA
    baseline(a) = prctile(gap(:,a), 20);
    b = gap(:,a) > baseline(a) + opts.blockRelDb;
    b = closeGaps(b, closeSamples);               % bridge brief dropouts first
    blocked(:,a) = enforceRuns(b, minSamples);     % then drop too-short runs
end
anyBlocked = any(blocked, 2);

fprintf('\n=== NLOS transient experiment: %s ===\n', logFile);
fprintf('%d sweeps, %.0f s, tag %d\n', height(T), t(end), opts.tagId);
fprintf('per-anchor clear-time gap baseline (dB): %s\n', ...
        mat2str(round(baseline,1)));
for a = 1:nA
    nb = sum(blocked(:,a));
    if nb == 0, continue; end
    fprintf('  A%d: blocked %.0f s (%.0f%% of run) - median gap %.1f dB blocked vs %.1f dB clear, ', ...
            A.ids(a), nb*dt, 100*nb/height(T), ...
            median(gap(blocked(:,a),a),'omitnan'), median(gap(~blocked(:,a),a),'omitnan'));
    fprintf('range %+.0f mm vs clear while blocked\n', ...
            1000*(median(rangeRaw(blocked(:,a),a),'omitnan') - median(rangeRaw(~blocked(:,a),a),'omitnan')));
end

%% ---- position comparison, weighted vs unweighted, IN the blocked windows --
if ~isempty(opts.truthXY)
    truth = opts.truthXY(:)';
    eW = vecnorm(Pw - truth, 2, 2); eU = vecnorm(Pu - truth, 2, 2);
    metric = 'error vs truth';
else
    truth = median(Pw, 1, 'omitnan');
    eW = vecnorm(Pw - truth, 2, 2); eU = vecnorm(Pu - truth, 2, 2);
    metric = 'spread around median (no truthXY given)';
end
fprintf('\nposition %s, mm:\n', metric);
fprintf('  %-10s %10s %10s\n', 'window', 'weighted', 'unweighted');
fprintf('  %-10s %10.0f %10.0f  (n=%d)\n', 'clear', ...
        1000*median(eW(~anyBlocked),'omitnan'), 1000*median(eU(~anyBlocked),'omitnan'), sum(~anyBlocked));
fprintf('  %-10s %10.0f %10.0f  (n=%d)\n', 'BLOCKED', ...
        1000*median(eW(anyBlocked),'omitnan'), 1000*median(eU(anyBlocked),'omitnan'), sum(anyBlocked));
fprintf('  %-10s %10.0f %10.0f  (p95)\n', 'BLOCKED', ...
        1000*prctile(eW(anyBlocked),95), 1000*prctile(eU(anyBlocked),95));

out = struct('t', t, 'Pw', Pw, 'Pu', Pu, 'gap', gap, 'rangeRaw', rangeRaw, ...
             'wUsed', wUsed, 'blocked', blocked, 'eW', eW, 'eU', eU, 'A', A);

%% ---- plot ------------------------------------------------------------------
fig = figure('Position',[30 30 1400 780]);
tiledlayout(3,1,'TileSpacing','compact','Padding','compact');

nexttile; hold on; grid on;
cmap = lines(nA);
for a = 1:nA
    plot(t, gap(:,a), '-', 'Color', cmap(a,:), 'DisplayName', sprintf('A%d', A.ids(a)));
    yline(baseline(a) + opts.blockRelDb, '--', 'Color', cmap(a,:), 'HandleVisibility','off');
end
shadeBlocked(t, anyBlocked);
legend('Location','eastoutside'); ylabel('rx-fp gap (dB)');
title('physical signature: NLOS gap per anchor (shaded = auto-detected blocked)');

nexttile; hold on; grid on;
for a = 1:nA
    plot(t, wUsed(:,a), '-', 'Color', cmap(a,:), 'DisplayName', sprintf('A%d', A.ids(a)));
end
shadeBlocked(t, anyBlocked);
ylim([0 1.05]); ylabel('trust weight w(g)');
title('gapWeights.m output: the blocked anchor is de-emphasised, not dropped');

nexttile; hold on; grid on;
plot(t, 1000*eU, '-', 'Color',[.7 .3 .3], 'DisplayName','unweighted (NLOS off)');
plot(t, 1000*eW, '-', 'Color',[.2 .5 .2], 'DisplayName','gap-weighted (NLOS on)');
shadeBlocked(t, anyBlocked);
legend('Location','eastoutside'); xlabel('t (s)'); ylabel(['position ' metric ' (mm)']);
title('the protection: weighted vs unweighted position, same raw data');

[dirp, base] = fileparts(logFile);
f = fullfile(dirp, [char(base) '_nlos_experiment.png']);
saveas(fig, f);
fprintf('\nfigure: %s\n', f);
out.figFile = f;
end

%% ── helpers ─────────────────────────────────────────────────────────────
function p = resolveLogPath(p)
% Accept an as-given path, one relative to the project root (parent of
% matlab/), one relative to matlab/, or bare filename under matlab/logs -
% covers running from any current directory.
if isfile(p), return; end
root = dune.rootDir();               % .../UWB_3/matlab
projRoot = fileparts(root);          % .../UWB_3
[~, base, ext] = fileparts(p);
cands = [fullfile(projRoot, p), fullfile(root, p), ...
         fullfile(root, 'logs', [char(base) char(ext)])];
for c = cands
    if isfile(c), p = char(c); return; end
end
error('nlos_transient_experiment:notFound', ...
      'Cannot find %s. Tried:\n  %s', p, strjoin(cands, '\n  '));
end

function b = enforceRuns(b, minLen)
% Zero out any TRUE run shorter than minLen samples (debounce).
n = numel(b); i = 1;
while i <= n
    if ~b(i), i = i+1; continue; end
    j = i; while j <= n && b(j), j = j+1; end
    if (j - i) < minLen, b(i:j-1) = false; end
    i = j;
end
end

function b = closeGaps(b, maxGapLen)
% Morphological "closing": fill any FALSE run of length <= maxGapLen that is
% sandwiched between TRUE on both sides. Real blocking is rarely a clean
% constant gap reading - a hand/body shifting mid-block can dip below
% threshold for a sample or two - so bridge those before the min-run filter,
% or a single real block fragments into many too-short pieces and vanishes.
if maxGapLen <= 0, return; end
n = numel(b); i = 1;
while i <= n
    if b(i), i = i+1; continue; end
    j = i; while j <= n && ~b(j), j = j+1; end
    if i > 1 && j <= n && (j - i) <= maxGapLen
        b(i:j-1) = true;
    end
    i = j;
end
end

function shadeBlocked(t, mask)
d = diff([0; double(mask); 0]);
onI = find(d == 1); offI = find(d == -1) - 1;
yl = ylim;
for k = 1:numel(onI)
    patch(t([onI(k) offI(k) offI(k) onI(k)]), yl([1 1 2 2]), ...
          [1 0.85 0.85], 'EdgeColor','none', 'FaceAlpha',0.5, 'HandleVisibility','off');
end
ylim(yl);
end
