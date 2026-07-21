function rigid_replay(rawLog, L, opts)
%RIGID_REPLAY Compare independent vs rigid-body solving on a two-tag UDP log.
%   rigid_replay                         % newest udp_raw_*.log, L = 0.315 m
%   rigid_replay(path)                   % a specific udp_raw_*.log
%   rigid_replay(path, 0.30)             % set the hard baseline L (m)
%   rigid_replay(path, 0.30, plot=true)  % also plot centre + yaw traces
%
%   Replays a raw UDP capture (thost <TAB> srcIp <TAB> RTLS line) written by
%   live_dual_tag's rawLogFid. Pairs the two tags by nearest t_host, then for
%   each pair runs (a) two independent dune.solveSweep fixes and (b) one
%   dune.rigidSolve with the inter-tag distance fixed at L. Reports how much the
%   rigid constraint tightens the baseline, the centre wander and the yaw.
%
%   Use it to sanity-check the rigid solver before wiring it into the live map,
%   and to pick L (independent baseline mean/median is printed).

arguments
    rawLog string = ""
    L (1,1) double = 0.315
    opts.tagZ (1,1) double = 0.24
    opts.plot (1,1) logical = false
    opts.maxPairDt (1,1) double = 0.3   % s, nearest-time pairing window
end

if rawLog == ""
    d = dir(fullfile(dune.rootDir(), 'logs', 'udp_raw_*.log'));
    assert(~isempty(d), 'no udp_raw_*.log in logs/');
    [~, ix] = max([d.datenum]);
    rawLog = string(fullfile(d(ix).folder, d(ix).name));
end
fprintf('replay: %s\n  L = %.3f m, tagZ = %.2f m\n', rawLog, L, opts.tagZ);

A  = dune.loadAnchors();
RC = dune.loadRangeCorrection();

% ---- parse both tags' sweeps from the raw capture ----
lines = readlines(rawLog); lines(strlength(strtrim(lines))==0) = [];
byTag = containers.Map('KeyType','double','ValueType','any');
for i = 1:numel(lines)
    parts = split(lines(i), sprintf('\t'));
    if numel(parts) < 3, continue; end
    s = dune.parseRtlsLine(parts(3));
    if isempty(s), continue; end
    s.thost = double(parts(1));
    if ~byTag.isKey(s.tag), byTag(s.tag) = {}; end
    q = byTag(s.tag); q{end+1} = s; byTag(s.tag) = q; %#ok<AGROW>
end
ids = sort(cell2mat(byTag.keys));
assert(numel(ids) >= 2, 'need two tags in the log (found %d)', numel(ids));
idLo = ids(1); idHi = ids(2);           % psi points toward the higher id
loSw = byTag(idLo); hiSw = byTag(idHi);
tHi = cellfun(@(s) s.thost, hiSw);
fprintf('  tags: lo=%d (%d sweeps), hi=%d (%d sweeps)\n', ...
        idLo, numel(loSw), idHi, numel(hiSw));

% ---- pair each lo sweep with the nearest-in-time hi sweep, solve both ways ----
R = struct('t',{},'blenInd',{},'cInd',{},'yawInd',{}, ...
           'c',{},'yaw',{},'rmseInd',{},'rmse',{});
prevPsi = [];
for k = 1:numel(loSw)
    sLo = loSw{k};
    [dt, j] = min(abs(tHi - sLo.thost));
    if dt > opts.maxPairDt, continue; end
    sHi = hiSw{j};

    o = dune.rigidSolve(sHi, sLo, A, L, rangeCorr=RC, tagZ=opts.tagZ, psi0=prevPsi);
    if ~o.ok, continue; end
    prevPsi = o.psi;

    rec.t = sLo.thost;
    if all(isfinite(o.pHiInd)) && all(isfinite(o.pLoInd))
        rec.blenInd = norm(o.pHiInd - o.pLoInd);
        rec.cInd    = (o.pHiInd + o.pLoInd) / 2;
        rec.yawInd  = atan2d(o.pHiInd(2)-o.pLoInd(2), o.pHiInd(1)-o.pLoInd(1));
    else
        rec.blenInd = NaN; rec.cInd = [NaN NaN]; rec.yawInd = NaN;
    end
    rec.c    = o.c;
    rec.yaw  = o.yawDeg;
    rmseIndParts = [rmseOf(sHi,A,RC,opts.tagZ), rmseOf(sLo,A,RC,opts.tagZ)];
    rec.rmseInd = mean(rmseIndParts(isfinite(rmseIndParts)));
    rec.rmse    = o.rmse;
    R(end+1) = rec; %#ok<AGROW>
end
n = numel(R);
fprintf('  paired & solved: %d\n\n', n);
if n < 2, fprintf('too few pairs\n'); return; end

blenInd = [R.blenInd];
cInd = reshape([R.cInd],2,[]).';  cRig = reshape([R.c],2,[]).';
yawInd = [R.yawInd]; yawRig = [R.yaw];
rmseInd = [R.rmseInd]; rmseRig = [R.rmse];

fprintf('=== baseline length ===\n');
fprintf('  independent : mean %.4f m  std %.1f mm  p2p %.1f mm\n', ...
    mean(blenInd,'omitnan'), 1000*std(blenInd,'omitnan'), ...
    1000*(max(blenInd)-min(blenInd)));
fprintf('  rigid       : %.4f m  (fixed by construction)\n\n', L);

fprintf('=== centre wander (std about mean; parked => lower is better) ===\n');
fprintf('  independent : %.1f mm   rigid : %.1f mm   (%+.0f%%)\n\n', ...
    1000*centreStd(cInd), 1000*centreStd(cRig), ...
    100*(centreStd(cRig)/centreStd(cInd)-1));

fprintf('=== yaw ===\n');
fprintf('  independent : mean %.1f  std %.2f deg\n', meanAng(yawInd), stdAng(yawInd));
fprintf('  rigid       : mean %.1f  std %.2f deg   (%+.0f%%)\n\n', ...
    meanAng(yawRig), stdAng(yawRig), 100*(stdAng(yawRig)/stdAng(yawInd)-1));

fprintf('=== range-residual rmse ===\n');
fprintf('  independent : %.1f mm    rigid : %.1f mm\n', ...
    1000*mean(rmseInd,'omitnan'), 1000*mean(rmseRig,'omitnan'));

if opts.plot
    figure('Name','rigid_replay');
    subplot(2,1,1); hold on; grid on; axis equal;
    plot(cInd(:,1), cInd(:,2), '.-', 'Color',[.8 .4 .4], 'DisplayName','indep centre');
    plot(cRig(:,1), cRig(:,2), '.-', 'Color',[.2 .4 .8], 'DisplayName','rigid centre');
    legend; title('centre trace'); xlabel x; ylabel y;
    subplot(2,1,2); hold on; grid on;
    plot(yawInd, '.-', 'Color',[.8 .4 .4], 'DisplayName','indep yaw');
    plot(yawRig, '.-', 'Color',[.2 .4 .8], 'DisplayName','rigid yaw');
    legend; title('yaw (deg)'); xlabel('pair #');
end
end

% ---------------------------------------------------------------------------
function e = rmseOf(s, A, RC, z)
[~, info] = dune.solveSweep(s, A, rangeCorr=RC, tagZ=z);
e = info.rmse;
end
function s = centreStd(C)
s = sqrt(mean(sum((C - mean(C,1,'omitnan')).^2, 2), 'omitnan'));
end
function m = meanAng(a), a = a(isfinite(a)); m = atan2d(mean(sind(a)), mean(cosd(a))); end
function s = stdAng(a)
a = a(isfinite(a)); a = a - meanAng(a); a = mod(a+180,360)-180; s = std(a);
end
