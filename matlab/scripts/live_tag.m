function live_tag(port, opts)
%LIVE_TAG Live UWB tag position from the COM stream (step 3).
%   live_tag                 % auto-detect port (needs exactly one free COM)
%   live_tag("COM7")
%   live_tag("COM7", bias=true)   % ALSO subtract host-side anchor_bias.json
%
%   Streams RTLS sweeps from the tag over serial, solves each one with
%   dune.solveSweep (sentinel reject -> NLOS gap weights -> weighted LM) and
%   shows a live floor map: anchors, current tag position, recent trail, and
%   a rate / anchors-used / residual readout. Close the figure to stop.
%
%   Every sweep is logged as JSONL (dune.readSessionLog schema) to
%   matlab\logs\rtls_log_<stamp>.jsonl, and every raw serial line to
%   serial_raw_<stamp>.log, so any live run can be replayed later.
%
%   On the first sweep from a tag, GETMYDELAY is sent; the MYDELAY_ACK reply
%   (the tag's active NVS antenna delay, in ticks) is printed with all other
%   non-RTLS device lines as "[dev] ...".
%
%   Host-side bias defaults OFF: the boards' NVS antenna delays are the
%   source of truth (re-tuned in step 4); subtracting anchor_bias.json on
%   top would double-correct.
%
%   opts: bias (false), tagZ (0.22 m), trail (300 fixes), logDir (matlab\logs)

arguments
    port string = ""
    opts.bias (1,1) logical = false
    opts.powerCorr (1,1) logical = true   % DW1000 power-bias correction
    opts.ekf (1,1) logical = true         % FusionEkf smoothing on the display
    opts.mode string = "pos"              % "pos" = fix updates (robust);
                                          % "ranges" = tightly-coupled with
                                          % per-anchor bias MEMORY: no position
                                          % jump when the anchor subset changes
                                          % (dropouts/skips). Best while parked
                                          % or slow; may lag when carried fast.
    opts.tagZ (1,1) double = 0.24         % tag antenna height (matches truth clicks)
    opts.trail (1,1) double = 300
    opts.margin (1,1) double = 2.0   % plot margin around the anchors (m)
    opts.logDir string = ""
end

A = dune.loadAnchors();
B = [];
if opts.bias, B = dune.loadAnchorBias(); end
RC = [];
if opts.powerCorr, RC = dune.loadRangeCorrection(); end

%% Log files
if opts.logDir == "", opts.logDir = fullfile(dune.rootDir(), 'logs'); end
if ~exist(opts.logDir, 'dir'), mkdir(opts.logDir); end
stamp = datestr(now, 'yyyymmdd_HHMMSS'); %#ok<TNOW1,DATST>
logFile = fullfile(opts.logDir, ['rtls_log_' stamp '.jsonl']);
fid = fopen(logFile, 'w');

%% Serial
ts = dune.TagSerial(port);
ts.rawLogFid = fopen(fullfile(opts.logDir, ['serial_raw_' stamp '.log']), 'w');
cleanup = onCleanup(@() endSession(ts, fid, logFile));
ts.start();
fprintf('Listening on %s @ %d baud (%s, host bias %s, power corr %s)\n', ...
        ts.port, ts.baud, A.layout, string(opts.bias), ...
        string(~isempty(RC)));
fprintf('Logging to %s\n', logFile);

%% Figure
fig = figure('Name', sprintf('DUNE live tag - %s', ts.port), 'NumberTitle', 'off');
ax = axes(fig); hold(ax, 'on'); axis(ax, 'equal'); grid(ax, 'on');
plot(ax, A.pos(:, 1), A.pos(:, 2), 'k^', 'MarkerFaceColor', 'y', 'MarkerSize', 10);
text(ax, A.pos(:, 1) + 0.05, A.pos(:, 2), compose('A%d', A.ids));
xlim(ax, [min(A.pos(:, 1)) - opts.margin, max(A.pos(:, 1)) + opts.margin]);
ylim(ax, [min(A.pos(:, 2)) - opts.margin, max(A.pos(:, 2)) + opts.margin]);
xlabel(ax, 'x (m)'); ylabel(ax, 'y (m)');
trailH = plot(ax, nan, nan, '.-', 'Color', [0.35 0.55 0.9], 'MarkerSize', 6);
rawH   = plot(ax, nan, nan, 'o', 'Color', [0.65 0.65 0.65], 'MarkerSize', 6);
dotH   = plot(ax, nan, nan, 'o', 'MarkerFaceColor', 'r', 'MarkerEdgeColor', 'r', ...
              'MarkerSize', 9);
ttl = title(ax, 'waiting for RTLS stream...');

%% Main loop
trail = nan(opts.trail, 2);
prevPos = [];
tRate = [];
nSweeps = 0; nSolved = 0; nRejected = 0;
queried = [];
ekf = dune.FusionEkf();
if opts.mode == "ranges"
    ekf.enableRangeBias(numel(A.ids));   % per-anchor bias memory
end
tPrevEkf = NaN;
histA = nan(1, 8); histG = nan(1, 8);   % rolling stillness window (IMU ZUPT)
stillCnt = 0;                            % UWB-only stillness (no-IMU tags)
divergeStreak = 0;

while ishandle(fig)
    for e = ts.drainEvents()
        fprintf('[dev] %s\n', e{1}.line);
    end
    for c = ts.drain()
        s = c{1};
        nSweeps = nSweeps + 1;
        if ~ismember(s.tag, queried)
            queried(end+1) = s.tag; %#ok<AGROW>
            ts.send(sprintf('GETMYDELAY,%d', s.tag));   % report NVS delay state
        end
        [p, info] = dune.solveSweep(s, A, bias=B, rangeCorr=RC, ...
                                    tagZ=opts.tagZ, x0=prevPos);
        nRejected = nRejected + nnz(info.rejected);

        % EKF smoothing (CV predict + gated position update + windowed ZUPT)
        pe = [NaN, NaN];
        if opts.ekf
            dt = 0;
            if isfinite(tPrevEkf), dt = min(max(s.thost - tPrevEkf, 0), 1); end
            tPrevEkf = s.thost;
            % Stillness detection BEFORE predict: still -> ZUPT + frozen
            % process noise, so the state can truly pin while parked.
            if ~isempty(s.imu) && s.imu.status >= 1
                histA = [histA(2:end), norm(s.imu.acc)];
                histG = [histG(2:end), norm(s.imu.gyro)];
                still = all(isfinite(histA)) && ...
                        max(histA) < 0.12 && max(histG) < 0.05;
            else
                % No IMU on this tag: infer stillness from the filter itself
                % (calm innovations + near-zero velocity for ~1.5 s; real
                % motion breaks the condition within a sweep or two).
                still = stillCnt >= 10;
            end
            ekf.stillMode = still;
            ekf.predict(dt, []);
            if opts.mode == "ranges"
                if ~ekf.initialized
                    if all(isfinite(p)), ekf.updatePosition(p); end
                else
                    ekf.updateRanges(A.pos, info.rangeCorr, info.w, ...
                                     opts.tagZ, 0.05);
                    solid = all(isfinite(p)) && nnz(info.used) >= 4 ...
                            && info.rmse < 0.10;
                    if solid && norm(ekf.pos - p) > 0.5
                        divergeStreak = divergeStreak + 1;
                    elseif solid
                        divergeStreak = 0;
                    end
                    if (ekf.consecReject >= ekf.maxConsecReject ...
                            || divergeStreak >= 5) && all(isfinite(p))
                        ekf.reinitFrom(p);
                        divergeStreak = 0;
                    end
                end
            elseif all(isfinite(p))
                ekf.updatePosition(p, adaptiveR(info, ekf.posSigma));
                if ekf.consecReject >= ekf.maxConsecReject
                    ekf.reinitFrom(p);
                end
            end
            if still && ekf.initialized
                ekf.updateZupt();
            end
            if isempty(s.imu) && ekf.initialized
                if norm(ekf.vel) < 0.08 && ekf.lastNis < 3
                    stillCnt = stillCnt + 1;
                else
                    stillCnt = 0;
                end
            end
            if ekf.initialized, pe = ekf.pos; end
        end
        fprintf(fid, '%s\n', jsonencode(dune.sweepRecord(s, p, info, A, pe)));

        disp_ = p;
        if opts.ekf && all(isfinite(pe)), disp_ = pe; end
        if all(isfinite(p)), set(rawH, 'XData', p(1), 'YData', p(2)); end
        if all(isfinite(p))
            nSolved = nSolved + 1;
            prevPos = p;
            trail = [trail(2:end, :); disp_];
            tRate(end+1) = s.thost; %#ok<AGROW>
            tRate(tRate < s.thost - 10) = [];
            set(trailH, 'XData', trail(:, 1), 'YData', trail(:, 2));
            set(dotH, 'XData', p(1), 'YData', p(2));
            % Grow the view if the tag wanders outside it
            xl = xlim(ax); yl = ylim(ax);
            if p(1) < xl(1) || p(1) > xl(2) || p(2) < yl(1) || p(2) > yl(2)
                xlim(ax, [min(xl(1), p(1) - 0.5), max(xl(2), p(1) + 0.5)]);
                ylim(ax, [min(yl(1), p(2) - 0.5), max(yl(2), p(2) + 0.5)]);
            end
            hz = NaN;
            if numel(tRate) > 1, hz = (numel(tRate) - 1) / (tRate(end) - tRate(1)); end
            mode = '';
            if opts.ekf && all(isfinite(pe)), mode = ' EKF'; end
            ttl.String = sprintf(['tag %d%s   (%.2f, %.2f) m   %.1f Hz   ' ...
                                  '%d/%d anchors   resid %.0f mm   solved %d/%d'], ...
                s.tag, mode, disp_(1), disp_(2), hz, nnz(info.used), ...
                nnz(~isnan(info.range)), 1000 * info.rmse, nSolved, nSweeps);
        else
            ttl.String = sprintf('tag %d   NO FIX (%d anchors usable)   solved %d/%d', ...
                s.tag, nnz(~isnan(info.range)), nSolved, nSweeps);
        end
    end
    drawnow limitrate
    pause(0.05);
end

fprintf('Figure closed: %d sweeps, %d solved, %d sentinel-rejected measurements.\n', ...
        nSweeps, nSolved, nRejected);
end

function endSession(ts, fid, logFile)
delete(ts);            % stops the callback, releases COM, closes raw log
fclose(fid);
fprintf('Session log: %s\n', logFile);
end

function R = adaptiveR(info, posSigma)
% LM covariance inflated by solver RMSE, mean NLOS gap, and a DOP proxy.
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
