function rec = sweepRecord(s, p, info, A)
%SWEEPRECORD One solved sweep -> JSONL-ready struct (readSessionLog schema).
%   rec = dune.sweepRecord(sweep, pos, info, A)
%   Write with: fprintf(fid, '%s\n', jsonencode(rec));
%
%   Fields mirror the historical Python logger so dune.readSessionLog and
%   replay tooling work unchanged on live-MATLAB session logs:
%     t_host, t_ms, tag_id, r<id> (raw m), d<id> (corrected m), x, y,
%     nUsed, rmse, anchor_diag [{id, rx, fp, gap, w, used, rejected}],
%     imu_status/imu_quat/imu_gyro when the sweep carries an IMU tail.

rec = struct('t_host', s.thost, 't_ms', s.tms, 'tag_id', s.tag);
for c = 1:numel(A.ids)
    if isfinite(info.range(c))
        rec.(sprintf('r%d', A.ids(c))) = round(info.range(c), 4);
        rec.(sprintf('d%d', A.ids(c))) = round(info.rangeCorr(c), 4);
    end
end
if all(isfinite(p))
    rec.x = round(p(1), 4);
    rec.y = round(p(2), 4);
end
rec.nUsed = nnz(info.used);
if isfinite(info.rmse), rec.rmse = round(info.rmse, 4); end

haveDiag = find(isfinite(info.range) | info.rejected)';
if ~isempty(haveDiag)
    diag = struct('id', {}, 'rx', {}, 'fp', {}, 'gap', {}, 'w', {}, ...
                  'used', {}, 'rejected', {});
    for c = haveDiag
        diag(end+1) = struct('id', A.ids(c), 'rx', info.rx(c), ...
            'fp', info.fp(c), 'gap', round(info.gap(c), 2), ...
            'w', round(info.w(c), 3), 'used', info.used(c), ...
            'rejected', info.rejected(c)); %#ok<AGROW>
    end
    rec.anchor_diag = diag;
end
if ~isempty(s.imu)
    rec.imu_status = s.imu.status;
    rec.imu_quat = s.imu.quat;
    rec.imu_gyro = s.imu.gyro;
end
end
