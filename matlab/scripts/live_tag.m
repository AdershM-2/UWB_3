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
    opts.tagZ (1,1) double = 0.22
    opts.trail (1,1) double = 300
    opts.margin (1,1) double = 2.0   % plot margin around the anchors (m)
    opts.logDir string = ""
end

A = dune.loadAnchors();
B = [];
if opts.bias, B = dune.loadAnchorBias(); end

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
fprintf('Listening on %s @ %d baud (%s, host bias %s)\n', ...
        ts.port, ts.baud, A.layout, string(opts.bias));
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
dotH   = plot(ax, nan, nan, 'o', 'MarkerFaceColor', 'r', 'MarkerEdgeColor', 'r', ...
              'MarkerSize', 9);
ttl = title(ax, 'waiting for RTLS stream...');

%% Main loop
trail = nan(opts.trail, 2);
prevPos = [];
tRate = [];
nSweeps = 0; nSolved = 0; nRejected = 0;
queried = [];

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
        [p, info] = dune.solveSweep(s, A, bias=B, tagZ=opts.tagZ, x0=prevPos);
        nRejected = nRejected + nnz(info.rejected);
        fprintf(fid, '%s\n', jsonencode(dune.sweepRecord(s, p, info, A)));
        if all(isfinite(p))
            nSolved = nSolved + 1;
            prevPos = p;
            trail = [trail(2:end, :); p];
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
            ttl.String = sprintf(['tag %d   (%.2f, %.2f) m   %.1f Hz   ' ...
                                  '%d/%d anchors   resid %.0f mm   solved %d/%d'], ...
                s.tag, p(1), p(2), hz, nnz(info.used), nnz(~isnan(info.range)), ...
                1000 * info.rmse, nSolved, nSweeps);
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
