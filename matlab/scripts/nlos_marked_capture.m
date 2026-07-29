function sidecarFile = nlos_marked_capture(port, opts)
%NLOS_MARKED_CAPTURE Interactive NLOS capture with hand-marked phase boundaries.
%   sidecarFile = nlos_marked_capture("COM12")
%   sidecarFile = nlos_marked_capture("COM12", nBlockPhases=3)
%
%   Walks you through: a CLEAR baseline phase (all anchors in line of sight),
%   then N block phases where YOU tell it which anchor you're about to block
%   and press Enter to mark the start/end of each phase. Raw sweeps (with
%   full rx/fp diagnostics) are logged continuously throughout via
%   dune.TagSerial's line callback, which keeps running even while this
%   script is blocked waiting on your Enter key - so nothing is lost between
%   phase markers.
%
%   Ground-truth phase timestamps beat auto-detecting "blocked" from the gap
%   signal (what nlos_transient_experiment.m does): no threshold-tuning, no
%   ambiguity about which anchor was targeted, no fragmenting on a noisy
%   block gesture.
%
%   Analyze the result with: nlos_marked_report(sidecarFile)

arguments
    port string = ""
    opts.tagId (1,1) double = 240
    opts.nBlockPhases (1,1) double = 2
    opts.logDir string = ""
end

if opts.logDir == "", opts.logDir = fullfile(dune.rootDir(), 'logs'); end
if ~exist(opts.logDir, 'dir'), mkdir(opts.logDir); end
stamp = datestr(now, 'yyyymmdd_HHMMSS'); %#ok<TNOW1,DATST>
rawFile = fullfile(opts.logDir, sprintf('nlos_raw_%s.log', stamp));

ts = dune.TagSerial(port);
cleanup = onCleanup(@() delete(ts));
ts.rawLogFid = fopen(rawFile, 'w');
ts.start();
fprintf('Connected to %s. Tag rebooting...\n', ts.port);
pause(3);
ts.drain(); ts.drainEvents();   % flush boot noise, logging continues regardless

phases = struct('label', {}, 'anchorId', {}, 'startS', {}, 'endS', {});

fprintf('\n=== Phase: CLEAR (baseline) ===\n');
fprintf('Position the tag so ALL anchors have a clear line of sight.\n');
input('Ready. Press Enter to START this phase...', 's');
t0 = nowPosix();
input('Recording baseline. Press Enter to END this phase (you will block an anchor next)...', 's');
t1 = nowPosix();
phases(end+1) = struct('label', 'clear', 'anchorId', NaN, 'startS', t0, 'endS', t1);
fprintf('  clear phase: %.1f s\n', t1 - t0);

for k = 1:opts.nBlockPhases
    aid = input(sprintf('\nWhich anchor are you about to block (phase %d/%d)? Enter its id: ', ...
                         k, opts.nBlockPhases));
    fprintf('=== Phase: BLOCK A%d ===\n', aid);
    input(sprintf('Step into A%d''s direct path now. Press Enter to START this phase...', aid), 's');
    t0 = nowPosix();
    input('Hold the block steady. Press Enter to END this phase...', 's');
    t1 = nowPosix();
    phases(end+1) = struct('label', sprintf('block_A%d', aid), 'anchorId', aid, ...
                            'startS', t0, 'endS', t1);
    fprintf('  block A%d phase: %.1f s\n', aid, t1 - t0);
    if k < opts.nBlockPhases
        input('Step away / clear the path. Press Enter when ready for the next block...', 's');
    end
end

ts.stop();
sidecar = struct('rawLogFile', rawFile, 'tagId', opts.tagId, 'phases', phases);
sidecarFile = fullfile(opts.logDir, sprintf('nlos_marked_%s.json', stamp));
fh = fopen(sidecarFile, 'w');
fwrite(fh, jsonencode(sidecar, 'PrettyPrint', true));
fclose(fh);
fprintf('\nSaved: %s\nraw log: %s\n', sidecarFile, rawFile);
fprintf('Analyze with: nlos_marked_report("%s")\n', sidecarFile);
end

function s = nowPosix()
s = posixtime(datetime('now', 'TimeZone', 'UTC'));
end
