function A = loadAnchors(jsonPath)
%LOADANCHORS Load anchor geometry from anchors.json.
%   A.ids   [M x 1]  anchor short addresses (1..5)
%   A.pos   [M x 3]  anchor antenna positions (m), world/anchor frame
%   A.bounds [xmin xmax ymin ymax zmin zmax]
%   A.dim   configured solve dimension (2 -> planar solve, fixed tag z)

if nargin < 1 || isempty(jsonPath)
    jsonPath = fullfile(dune.rootDir(), 'config', 'anchors.json');
end
raw = jsondecode(fileread(jsonPath));

n = numel(raw.anchors);
A.ids = zeros(n, 1);
A.pos = zeros(n, 3);
for k = 1:n
    a = raw.anchors(k);
    A.ids(k)   = a.id;
    A.pos(k,:) = [a.x, a.y, a.z];
end
A.bounds = raw.bounds(:)';
A.dim    = raw.dim;
A.layout = raw.layout;
A.file   = jsonPath;
end
