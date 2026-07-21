function live_dual_tag(opts)
%LIVE_DUAL_TAG Two UWB tags over WiFi/UDP: positions + baseline yaw (step 7.3).
%   live_dual_tag                       % listen on UDP 4100, auto-discover tags
%   live_dual_tag(baseline=0.50)        % known antenna-to-antenna spacing (m)
%   live_dual_tag(ekf=false)            % raw solves, no per-tag smoothing
%
%   Both tags UDP-broadcast their RTLS sweeps (HostLink UDP). One dune.TagUdp
%   socket ingests BOTH streams; each sweep is demuxed by tag_id and solved
%   independently with dune.solveSweep (+ optional per-tag FusionEkf). The map
%   shows each tag as its own dot + trail and draws the rigid BASELINE between
%   them; the readout reports:
%     - each tag's (x,y),
%     - the measured inter-tag distance vs the known baseline (a rigid-body
%       sanity check and a live way to re-measure L),
%     - the baseline yaw  = atan2(tag_hi - tag_lo)  (tag ids sorted ascending),
%     - the IMU yaw from the IMU-bearing tag (BNO085 rotation vector),
%     - their offset + its circular-mean (quantifies the rotated IMU frame the
%       project suspects: a STABLE offset = a pure frame rotation).
%
%   This is transport #2 (UDP) so both tags run at once without USB. It is the
%   base for the rigid-body joint solve (task 4): the baseline distance/yaw
%   shown here become the constraint.
%
%   Every tag's sweep is logged as JSONL (dune.sweepRecord schema, replayable)
%   to logs/dual_log_<stamp>.jsonl; the per-sample baseline/yaw is logged to
%   logs/dual_yaw_<stamp>.jsonl.
%
%   opts: baseline (0.50 m), tagZ (0.24), ekf (true), powerCorr (true),
%         nlos (true), hold (true), port (4100), trail (300), margin (2.0),
%         logDir (logs)

arguments
    opts.baseline (1,1) double = 0.50     % known tag1<->tag2 antenna spacing (m)
    opts.tagZ (1,1) double = 0.24
    opts.ekf (1,1) logical = true         % per-tag FusionEkf smoothing
    opts.powerCorr (1,1) logical = true
    opts.nlos (1,1) logical = true
    opts.hold (1,1) logical = true        % per-tag RangeHold across dropouts
    opts.port (1,1) double = 4100
    opts.trail (1,1) double = 300
    opts.margin (1,1) double = 2.0
    opts.logDir string = ""
end

A = dune.loadAnchors();
RC = [];
if opts.powerCorr, RC = dune.loadRangeCorrection(); end

%% Log files
if opts.logDir == "", opts.logDir = fullfile(dune.rootDir(), 'logs'); end
if ~exist(opts.logDir, 'dir'), mkdir(opts.logDir); end
stamp = datestr(now, 'yyyymmdd_HHMMSS'); %#ok<TNOW1,DATST>
logFile  = fullfile(opts.logDir, ['dual_log_' stamp '.jsonl']);
yawFile  = fullfile(opts.logDir, ['dual_yaw_' stamp '.jsonl']);
fid    = fopen(logFile, 'w');
fidYaw = fopen(yawFile, 'w');

%% Transport (UDP, both tags on one socket)
tu = dune.TagUdp(opts.port);
tu.rawLogFid = fopen(fullfile(opts.logDir, ['udp_raw_' stamp '.log']), 'w');
cleanup = onCleanup(@() endSession(tu, fid, fidYaw, logFile));
tu.start();
fprintf('Listening on UDP :%d  (%s, power corr %s, baseline %.3f m)\n', ...
        opts.port, A.layout, string(~isempty(RC)), opts.baseline);
fprintf('Logging sweeps -> %s\n         yaw    -> %s\n', logFile, yawFile);

%% Figure
fig = figure('Name', 'DUNE live dual tag (UDP)', 'NumberTitle', 'off');
ax = axes(fig); hold(ax, 'on'); axis(ax, 'equal'); grid(ax, 'on');
plot(ax, A.pos(:,1), A.pos(:,2), 'k^', 'MarkerFaceColor', 'y', 'MarkerSize', 10);
text(ax, A.pos(:,1) + 0.05, A.pos(:,2), compose('A%d', A.ids));
xlim(ax, [min(A.pos(:,1)) - opts.margin, max(A.pos(:,1)) + opts.margin]);
ylim(ax, [min(A.pos(:,2)) - opts.margin, max(A.pos(:,2)) + opts.margin]);
xlabel(ax, 'x (m)'); ylabel(ax, 'y (m)');
baseH = plot(ax, nan, nan, '-', 'Color', [0.15 0.15 0.15], 'LineWidth', 2);
ttl = title(ax, 'waiting for tags on UDP...');

%% State — per-tag map, discovered as tags appear
palette = [0.85 0.20 0.20;    % tag 1 red
           0.20 0.45 0.85;    % tag 2 blue
           0.20 0.65 0.30;    % (more tags, unlikely)
           0.75 0.45 0.90];
tags = containers.Map('KeyType', 'double', 'ValueType', 'any');
offSin = 0; offCos = 0;   % circular-mean accumulator for the IMU-frame offset
nSweeps = 0;

while ishandle(fig)
    for e = tu.drainEvents()
        fprintf('[dev %s] %s\n', e{1}.srcIp, e{1}.line);
    end
    sweeps = tu.drain();
    if ~ishandle(fig), break; end   % window closed during socket I/O (the only
                                    % interruption point) -> stop before any
                                    % graphics update touches a deleted handle
    batchTags = [];
    for ci = 1:numel(sweeps)
        s = sweeps{ci};
        nSweeps = nSweeps + 1;
        if ~tags.isKey(s.tag)
            tags(s.tag) = newTagState(ax, palette, tags.Count);
            fprintf('[tag %d (0x%02X)] first sweep from %s\n', s.tag, s.tag, s.srcIp);
        end
        st = tags(s.tag);

        % Stillness (IMU-bearing tags only; used for ZUPT)
        still = false;
        if ~isempty(s.imu) && s.imu.status >= 1
            st.histA = [st.histA(2:end), norm(s.imu.acc)];
            st.histG = [st.histG(2:end), norm(s.imu.gyro)];
            still = all(isfinite(st.histA)) && ...
                    max(st.histA) < 0.12 && max(st.histG) < 0.05;
        end

        nHeld = 0;
        if opts.hold, [s, nHeld] = st.rh.apply(s, still); end %#ok<ASGLU>

        [p, info] = dune.solveSweep(s, A, rangeCorr=RC, tagZ=opts.tagZ, ...
                                    x0=st.prevPos, useGapWeights=opts.nlos);

        % Per-tag EKF smoothing (position mode)
        pe = [NaN, NaN];
        if opts.ekf
            dt = 0;
            if isfinite(st.tPrev), dt = min(max(s.thost - st.tPrev, 0), 1); end
            st.tPrev = s.thost;
            st.ekf.stillMode = still;
            st.ekf.predict(dt, []);
            if all(isfinite(p))
                st.ekf.updatePosition(p);
                if st.ekf.consecReject >= st.ekf.maxConsecReject
                    st.ekf.reinitFrom(p);
                end
            end
            if still && st.ekf.initialized, st.ekf.updateZupt(); end
            if st.ekf.initialized, pe = st.ekf.pos; end
        end

        po = p;
        if opts.ekf && all(isfinite(pe)), po = pe; end

        fprintf(fid, '%s\n', jsonencode(dune.sweepRecord(s, p, info, A, po)));

        if all(isfinite(po))
            st.pos = po; st.thost = s.thost; st.rmse = info.rmse;
            st.nUsed = nnz(info.used);
            st.trail = [st.trail(2:end, :); po];
            set(st.trailH, 'XData', st.trail(:,1), 'YData', st.trail(:,2));
            set(st.dotH, 'XData', po(1), 'YData', po(2));
        end
        if all(isfinite(p)), st.prevPos = p; end
        if ~isempty(s.imu), st.yawImu = yawFromQuat(s.imu.quat); st.hasImu = true; end

        tags(s.tag) = st;
        if ~ismember(s.tag, batchTags), batchTags(end+1) = s.tag; end %#ok<AGROW>
    end

    % Baseline + yaw when >=2 tags have a recent fix
    if tags.Count >= 2 && ~isempty(batchTags)
        ids = sort(cell2mat(tags.keys));
        stLo = tags(ids(1)); stHi = tags(ids(2));
        fresh = @(st) isfield(st,'thost') && ~isempty(st.thost) && ...
                      (posixtime(datetime('now','TimeZone','UTC')) - st.thost) < 1.0;
        if all(isfinite(stLo.pos)) && all(isfinite(stHi.pos)) && ...
                fresh(stLo) && fresh(stHi)
            v = stHi.pos - stLo.pos;
            baseLen = hypot(v(1), v(2));
            yawBase = atan2d(v(2), v(1));
            set(baseH, 'XData', [stLo.pos(1) stHi.pos(1)], ...
                       'YData', [stLo.pos(2) stHi.pos(2)]);

            % IMU yaw from whichever tag carries the IMU
            yawImu = NaN;
            if stLo.hasImu, yawImu = stLo.yawImu; end
            if stHi.hasImu, yawImu = stHi.yawImu; end
            offStr = '';
            if isfinite(yawImu)
                off = wrap180(yawBase - yawImu);
                offSin = offSin + sind(off); offCos = offCos + cosd(off);
                offMean = atan2d(offSin, offCos);
                offStr = sprintf('  IMUyaw %+6.1f  off %+6.1f (mean %+6.1f)', ...
                                 yawImu, off, offMean);
            end

            rec = struct('t_host', max(stLo.thost, stHi.thost), ...
                'x1', round(stLo.pos(1),4), 'y1', round(stLo.pos(2),4), ...
                'x2', round(stHi.pos(1),4), 'y2', round(stHi.pos(2),4), ...
                'idLo', ids(1), 'idHi', ids(2), ...
                'baseLen', round(baseLen,4), 'yawBase', round(yawBase,2));
            if isfinite(yawImu), rec.yawImu = round(yawImu,2); end
            fprintf(fidYaw, '%s\n', jsonencode(rec));

            ttl.String = sprintf(['T%d (%.2f,%.2f)  T%d (%.2f,%.2f)   ' ...
                'base %.3f m (L %.3f, d%+.0fmm)   yaw %+6.1f%s'], ...
                ids(1), stLo.pos(1), stLo.pos(2), ids(2), stHi.pos(1), stHi.pos(2), ...
                baseLen, opts.baseline, 1000*(baseLen-opts.baseline), yawBase, offStr);
        end
    elseif tags.Count == 1 && ~isempty(batchTags)
        st = tags(batchTags(1));
        ttl.String = sprintf('tag %d (%.2f, %.2f)  resid %.0f mm  %d/5 anch   [1 of 2 tags - power the other]', ...
            batchTags(1), st.pos(1), st.pos(2), 1000*st.rmse, st.nUsed);
    end

    drawnow limitrate
    pause(0.05);
end

fprintf('Figure closed: %d sweeps across %d tag(s).\n', nSweeps, tags.Count);
end

% ---------------------------------------------------------------------------
function st = newTagState(ax, palette, idx)
col = palette(mod(idx, size(palette,1)) + 1, :);
st.col = col;
st.trailH = plot(ax, nan, nan, '.-', 'Color', min(col+0.35,1), 'MarkerSize', 6);
st.dotH   = plot(ax, nan, nan, 'o', 'MarkerFaceColor', col, ...
                 'MarkerEdgeColor', col, 'MarkerSize', 9);
st.trail  = nan(300, 2);
st.prevPos = [];
st.pos = [NaN NaN];
st.thost = [];
st.tPrev = NaN;
st.rmse = NaN; st.nUsed = 0;
st.ekf = dune.FusionEkf();
st.rh  = dune.RangeHold();
st.histA = nan(1, 8); st.histG = nan(1, 8);
st.yawImu = NaN; st.hasImu = false;
end

function y = yawFromQuat(q)
% Yaw (deg) about world Z from the BNO085 rotation vector [qw qx qy qz].
% Matches the firmware quatToRPY yaw.
y = NaN;
if numel(q) ~= 4, return; end
nq = norm(q); if nq < 0.5, return; end
q = q / nq; qw=q(1); qx=q(2); qy=q(3); qz=q(4);
siny = 2*(qw*qz + qx*qy);
cosy = 1 - 2*(qy*qy + qz*qz);
y = atan2d(siny, cosy);
end

function w = wrap180(a)
w = mod(a + 180, 360) - 180;
end

function endSession(tu, fid, fidYaw, logFile)
delete(tu);
fclose(fid); fclose(fidYaw);
fprintf('Session log: %s\n', logFile);
end
