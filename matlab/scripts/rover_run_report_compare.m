function out = rover_run_report_compare(matFile, opts)
%ROVER_RUN_REPORT_COMPARE EKF (top row) vs rigid MHE (bottom row), same run.
%   out = rover_run_report_compare("...\rover_uwb_x.mat")
%
%   Runs both rover_run_report_ekf and rover_run_report (plot=false), then
%   draws their three trajectory panels stacked so the two estimators are
%   directly comparable on identical data/truth/axes.

arguments
    matFile string = ""
    opts.trimStartS (1,1) double = 0
    opts.trimEndS (1,1) double = 0
end

if matFile == ""
    d = dir(fullfile(dune.rootDir(), 'results', 'rover_runs', 'rover_uwb_*.mat'));
    assert(~isempty(d), 'no rover_uwb_*.mat found');
    [~, i] = max([d.datenum]);
    matFile = fullfile(d(i).folder, d(i).name);
end

A = dune.loadAnchors();
fprintf('=== EKF ===\n');
oE = rover_run_report_ekf(matFile, plot=false, ...
        trimStartS=opts.trimStartS, trimEndS=opts.trimEndS);
fprintf('\n=== MHE ===\n');
oM = rover_run_report(matFile, plot=false, ...
        trimStartS=opts.trimStartS, trimEndS=opts.trimEndS);

TRAJ_LIM = [-3 3 -2 2];
fig = figure('Position',[30 30 1500 620]);
tiledlayout(2,3,'TileSpacing','compact','Padding','compact');

% ---- row 1: EKF (blue) -----------------------------------------------------
row(A, oE, [0.2 0.4 0.85], [0.1 0.2 0.8], 'EKF', TRAJ_LIM);

% ---- row 2: MHE (red) ------------------------------------------------------
row(A, oM, [0.85 0.2 0.2], [0.85 0.2 0.2], 'rigid MHE', TRAJ_LIM);

sgtitle(sprintf('%s — EKF (top) vs rigid MHE (bottom)', matFile), 'Interpreter','none');

f = replace(char(matFile), '.mat', '_report_compare.png');
saveas(fig, f);
fprintf('\nfigure: %s\n', f);
out = struct('ekf', oE, 'mhe', oM, 'figFile', f);
end

function row(A, o, colFull, colFix, name, lim)
nexttile; hold on; grid on; axis equal;
plot(A.pos(:,1), A.pos(:,2), 'k^','MarkerFaceColor','y','MarkerSize',9);
text(A.pos(:,1)+0.05, A.pos(:,2), compose('A%d', A.ids), 'Clipping','on');
plot(o.Praw(:,1), o.Praw(:,2), '.', 'Color',[.75 .75 .75], 'MarkerSize',4);
if any(~isnan(o.Pose(:,1)))
    plot(o.Pose(:,1), o.Pose(:,2), '-', 'Color', colFix, 'LineWidth',1.2);
end
title(sprintf('%s: UWB frame (raw grey + estimate)', name));
xlabel('x (m)'); ylabel('y (m)'); lockTraj(lim);

nexttile; hold on; grid on; axis equal;
plot(o.cT(:,1), o.cT(:,2), 'b.-', 'MarkerSize',5);
title(sprintf('%s: AprilTag truth (Kinect frame)', name));
xlabel('x (m)'); ylabel('y (m)'); lockTraj(lim);

nexttile; hold on; grid on; axis equal;
if isfield(o,'Ealigned')
    plot(o.PoseAligned(:,1), o.PoseAligned(:,2), '-', 'Color', colFull*0.4+0.6, 'LineWidth',0.8);
    plot(o.cTm(:,1), o.cTm(:,2), 'b-', 'LineWidth',1.4);
    plot(o.Ealigned(:,1), o.Ealigned(:,2), '-', 'Color', colFull, 'LineWidth',1.1);
    legend({[name ' full rate'],'truth',[name ' @ truth times']}, 'Location','best');
    title(sprintf('%s aligned: RMSE %.0f mm, path %.1fx truth', ...
                  name, 1000*sqrt(mean(o.err.^2)), o.pathRatio));
else
    title(sprintf('%s: not enough overlap to align', name));
end
xlabel('x (m)'); ylabel('y (m)'); lockTraj(lim);
end

function lockTraj(lim)
axis(lim); daspect([1 1 1]); pbaspect([diff(lim(1:2)) diff(lim(3:4)) 1]);
end
