function out = nlos_marked_report(sidecarFile, opts)
%NLOS_MARKED_REPORT Score an nlos_marked_capture session against its
%   hand-marked phase boundaries (ground truth - no gap-threshold guessing).
%
%   out = nlos_marked_report("...\nlos_marked_x.json")
%
%   Replays the raw log TWICE (dune.solveSweep, useGapWeights on/off - a pure
%   software toggle over the same raw ranges/diagnostics), then scores each
%   phase using its OWN marked [startS, endS] window: the "clear" phase is
%   the per-anchor baseline, each "block_A<id>" phase reports that anchor's
%   gap/range shift vs baseline plus the weighted-vs-unweighted position
%   spread while it was blocked.

arguments
    sidecarFile string
    opts.tagZ (1,1) double = 0.24
    opts.truthXY double = []   % [x y], optional - absolute error instead of spread
end

sidecarFile = resolvePath(sidecarFile);
Sc = jsondecode(fileread(sidecarFile));
rawFile = resolvePath(string(Sc.rawLogFile));
phases = Sc.phases;
if isstruct(phases) && ~isvector(phases), phases = phases(:); end
tagId = Sc.tagId;

A = dune.loadAnchors();
RC = dune.loadRangeCorrection();
nA = numel(A.ids);

%% ---- parse the raw tee log (tab-stamped thost + raw device line) ----------
raw = readlines(rawFile);
thostAbs = []; sw = {};
for i = 1:numel(raw)
    parts = split(raw(i), sprintf('\t'));
    if numel(parts) < 2, continue; end
    s = dune.parseRtlsLine(parts(2));
    if isempty(s) || s.tag ~= tagId, continue; end
    thostAbs(end+1) = double(parts(1)); %#ok<AGROW>
    sw{end+1} = s; %#ok<AGROW>
end
N = numel(sw);
assert(N > 20, 'not enough sweeps for tag %d in %s', tagId, rawFile);
t0abs = thostAbs(1);
t = thostAbs - t0abs;
fprintf('\n=== NLOS marked-phase experiment: %s ===\n', sidecarFile);
fprintf('%d sweeps, %.0f s, tag %d, %d phases\n', N, t(end), tagId, numel(phases));

%% ---- replay every sweep TWICE: gap-weighted vs unweighted -----------------
Pw = nan(N,2); Pu = nan(N,2);
gap = nan(N,nA); rangeRaw = nan(N,nA); wUsed = nan(N,nA);
prevW = []; prevU = [];
for i = 1:N
    s = sw{i};
    if numel(s.ids) < 3
        continue;
    end
    sweep = struct('tag', s.tag, 'ids', s.ids, 'dist', s.dist, 'rx', s.rx, 'fp', s.fp);
    [pw, infoW] = dune.solveSweep(sweep, A, rangeCorr=RC, tagZ=opts.tagZ, x0=prevW, ...
                                  useGapWeights=true);
    [pu, ~]     = dune.solveSweep(sweep, A, rangeCorr=RC, tagZ=opts.tagZ, x0=prevU, ...
                                  useGapWeights=false);
    Pw(i,:) = pw; if all(isfinite(pw)), prevW = pw; end
    Pu(i,:) = pu; if all(isfinite(pu)), prevU = pu; end
    gap(i,:) = infoW.gap'; rangeRaw(i,:) = infoW.range'; wUsed(i,:) = infoW.w';
end

%% ---- position metric -------------------------------------------------------
if ~isempty(opts.truthXY)
    truth = opts.truthXY(:)';
    metric = 'error vs truth';
else
    truth = median(Pw, 1, 'omitnan');
    metric = 'spread around median (no truthXY given)';
end
eW = vecnorm(Pw - truth, 2, 2); eU = vecnorm(Pu - truth, 2, 2);

%% ---- score each marked phase on its OWN window ----------------------------
clearIdx = [];
baseGap = nan(1, nA);
for k = 1:numel(phases)
    ph = phases(k);
    if strcmp(ph.label, 'clear')
        clearIdx = find(thostAbs >= ph.startS & thostAbs <= ph.endS);
        for a = 1:nA, baseGap(a) = median(gap(clearIdx,a), 'omitnan'); end
    end
end

fprintf('\n%-12s %6s  %10s %10s  %10s %10s  %10s %10s\n', ...
        'phase', 'dur(s)', 'gap(dB)', 'vs clear', 'range(mm)', 'vs clear', 'pos-W(mm)', 'pos-U(mm)');
results = struct('label', {}, 'idx', {});
for k = 1:numel(phases)
    ph = phases(k);
    idx = find(thostAbs >= ph.startS & thostAbs <= ph.endS);
    results(end+1) = struct('label', ph.label, 'idx', idx); %#ok<AGROW>
    dur = ph.endS - ph.startS;
    if isnan(ph.anchorId)
        gTxt = '--'; gDelta = '--'; rTxt = '--'; rDelta = '--';
    else
        a = find(A.ids == ph.anchorId, 1);
        g = median(gap(idx,a), 'omitnan');
        r = median(rangeRaw(idx,a), 'omitnan');
        rClear = median(rangeRaw(clearIdx,a), 'omitnan');
        gTxt = sprintf('%.1f', g); gDelta = sprintf('%+.1f', g - baseGap(a));
        rTxt = sprintf('%.0f', 1000*r); rDelta = sprintf('%+.0f', 1000*(r - rClear));
    end
    pW = 1000*median(eW(idx), 'omitnan'); pU = 1000*median(eU(idx), 'omitnan');
    fprintf('%-12s %6.1f  %10s %10s  %10s %10s  %10.0f %10.0f\n', ...
            ph.label, dur, gTxt, gDelta, rTxt, rDelta, pW, pU);
end
fprintf('(gap/range columns are for the anchor THAT phase targeted; position is %s)\n', metric);

out = struct('t', t, 'Pw', Pw, 'Pu', Pu, 'gap', gap, 'rangeRaw', rangeRaw, ...
             'wUsed', wUsed, 'eW', eW, 'eU', eU, 'phases', phases, 'A', A, 'results', results);

%% ---- plot -------------------------------------------------------------------
fig = figure('Position',[30 30 1400 780]);
tiledlayout(3,1,'TileSpacing','compact','Padding','compact');
cmap = lines(nA);

nexttile; hold on; grid on;
for a = 1:nA
    plot(t, gap(:,a), '-', 'Color', cmap(a,:), 'DisplayName', sprintf('A%d', A.ids(a)));
end
shadePhases(thostAbs, phases);
legend('Location','eastoutside'); ylabel('rx-fp gap (dB)');
title('physical signature: NLOS gap per anchor (shaded = marked phase)');

nexttile; hold on; grid on;
for a = 1:nA
    plot(t, wUsed(:,a), '-', 'Color', cmap(a,:), 'DisplayName', sprintf('A%d', A.ids(a)));
end
shadePhases(thostAbs, phases);
ylim([0 1.05]); ylabel('trust weight w(g)');
title('gapWeights.m output: the blocked anchor is de-emphasised, not dropped');

nexttile; hold on; grid on;
plot(t, 1000*eU, '-', 'Color',[.7 .3 .3], 'DisplayName','unweighted (NLOS off)');
plot(t, 1000*eW, '-', 'Color',[.2 .5 .2], 'DisplayName','gap-weighted (NLOS on)');
shadePhases(thostAbs, phases);
legend('Location','eastoutside'); xlabel('t (s)'); ylabel(['position ' metric ' (mm)']);
title('the protection: weighted vs unweighted position, same raw data');

[dirp, base] = fileparts(sidecarFile);
f = fullfile(dirp, [char(base) '_report.png']);
saveas(fig, f);
fprintf('\nfigure: %s\n', f);
out.figFile = f;
end

%% ── helpers ─────────────────────────────────────────────────────────────
function p = resolvePath(p)
if isfile(p), return; end
root = dune.rootDir(); projRoot = fileparts(root);
[~, base, ext] = fileparts(p);
cands = [fullfile(projRoot, p), fullfile(root, p), fullfile(root, 'logs', [char(base) char(ext)])];
for c = cands
    if isfile(c), p = char(c); return; end
end
error('nlos_marked_report:notFound', 'Cannot find %s. Tried:\n  %s', p, strjoin(cands, '\n  '));
end

function shadePhases(thostAbs, phases)
yl = ylim; t0abs = thostAbs(1);
for k = 1:numel(phases)
    ph = phases(k);
    x0 = ph.startS - t0abs; x1 = ph.endS - t0abs;
    if strcmp(ph.label, 'clear'), c = [0.85 1 0.85]; else, c = [1 0.85 0.85]; end
    patch([x0 x1 x1 x0], yl([1 1 2 2]), c, 'EdgeColor','none', ...
          'FaceAlpha',0.5, 'HandleVisibility','off');
    text(mean([x0 x1]), yl(2), ph.label, 'HorizontalAlignment','center', ...
         'VerticalAlignment','top', 'FontSize',8, 'Interpreter','none');
end
ylim(yl);
end
