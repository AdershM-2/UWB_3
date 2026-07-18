function s = parseRtlsLine(line)
%PARSERTLSLINE Parse one live "RTLS,v3" ASCII line from the tag firmware.
%   s = dune.parseRtlsLine(line)
%
%   Format (HostLink.h):
%     RTLS,v3,<t_ms>,<tag_id>,<n>,{<id>,<d_mm>,<rx_dbm>,<fp_dbm>,<q>}*n
%          [,IMU,<status>,<qw>,<qx>,<qy>,<qz>,<ax>,<ay>,<az>,<gx>,<gy>,<gz>]
%
%   Returns [] for non-RTLS lines (survey/calibration chatter). Output:
%     s.tms, s.tag, s.ids [n], s.dist [n] (m), s.rx, s.fp, s.qual
%     s.imu = [] or struct(status, quat[4], acc[3], gyro[3])

s = [];
line = strtrim(line);
if ~startsWith(line, "RTLS,"), return; end
tok = split(string(line), ',');
if numel(tok) < 5 || tok(2) ~= "v3", return; end

s.tms = double(tok(3));
s.tag = double(tok(4));
n     = double(tok(5));
s.ids = nan(1,n); s.dist = nan(1,n); s.rx = nan(1,n);
s.fp  = nan(1,n); s.qual = nan(1,n);
p = 6;
for k = 1:n
    s.ids(k)  = double(tok(p));
    s.dist(k) = double(tok(p+1)) / 1000;   % mm -> m
    s.rx(k)   = double(tok(p+2));
    s.fp(k)   = double(tok(p+3));
    s.qual(k) = double(tok(p+4));
    p = p + 5;
end

s.imu = [];
if p <= numel(tok) && tok(p) == "IMU" && numel(tok) >= p + 11
    v = double(tok(p+1:p+11));
    s.imu = struct('status', v(1), 'quat', v(2:5)', ...
                   'acc', v(6:8)', 'gyro', v(9:11)');
end
end
