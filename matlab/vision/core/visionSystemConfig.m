function config = visionSystemConfig()
    %VISIONSYSTEMCONFIG Configuration parameters for AprilTag-based vision system
    %   Returns a struct containing all validated camera parameters, workspace
    %   dimensions, coordinate system definitions, and AprilTag specifications.
    %
    %   These parameters have been tested and validated for the MMS rover system.
    %   Last validated: October 2025
    %
    %   Usage:
    %     config = visionSystemConfig();
    %     intrinsics = config.camera.intrinsics;
    %     tagSize = config.apriltag.size;
    %
    %   Coordinate System Convention:
    %     World Frame: X-right(+), Y-forward(+), Z-up(+)
    %     Camera Frame: X-right(+), Y-down(+), Z-forward/depth(+)
    %     Camera is mounted overhead looking downward at ground plane

    config = struct();

    %% Camera Parameters (VALIDATED - ORIGINAL)
    % These parameters were tested and provide good accuracy:
    % - Distance measurement: 50-100mm error at 1m movements
    % - Rotation measurement: 3-5° error at 90° rotations
    % - Tested date: October 2025
    config.camera = struct();
    config.camera.model = 'Kinect v2';
    config.camera.imageSize = [1080, 1920];  % [height, width] in pixels
    config.camera.focalLength = [1050.0, 1050.0];  % [fx, fy] in pixels
    config.camera.principalPoint = [960.0, 540.0];  % [cx, cy] in pixels
    config.camera.radialDistortion = [0, 0];  % No distortion correction (works better)
    config.camera.tangentialDistortion = [0, 0];

    % Create MATLAB camera intrinsics object
    config.camera.intrinsics = cameraIntrinsics(...
        config.camera.focalLength, ...
        config.camera.principalPoint, ...
        config.camera.imageSize);

    % Camera mounting (calibrated 2025-12-01)
    config.camera.position = [0, 0, 3.43];  % [X, Y, Z] in meters (world frame)
    config.camera.height = 3.43;  % meters above ground
    config.camera.orientation = 'downward';  % Looking straight down at ground

    % Camera device settings
    config.camera.deviceName = 'kinect';
    config.camera.deviceID = 1;
    config.camera.colorFormat = 'BGR_1920x1080';
    config.camera.returnedColorSpace = 'rgb';

    %% World-frame extrinsics (camera -> surveyed testbed/UWB frame)
    % Produced by core/registerWorldFrame.m from AprilTag detections at
    % surveyed floor points. When present, transformPoseToWorld() uses this
    % rigid transform instead of the plumb-camera (nadir) assumption, so the
    % vision world frame coincides with the surveyed frame (e.g. UWB anchors).
    config.extrinsics = struct('available', false, 'R', [], 't', [], ...
                               'rmse_m', NaN, 'file', '');
    regFile = fullfile(fileparts(mfilename('fullpath')), '..', ...
                       'calibration_data', 'world_registration.mat');
    if exist(regFile, 'file')
        reg = load(regFile);
        config.extrinsics.available = true;
        config.extrinsics.R = reg.R_cam2world;
        config.extrinsics.t = reg.t_cam2world;
        config.extrinsics.rmse_m = reg.rmse_m;
        config.extrinsics.file = regFile;
        if isfield(reg, 'registeredDate')
            config.extrinsics.registeredDate = reg.registeredDate;
        end
    end

    %% AprilTag Parameters (VALIDATED)
    config.apriltag = struct();

    % Base rover tag (primary localization)
    config.apriltag.base = struct();
    config.apriltag.base.family = 'tag36h11';
    config.apriltag.base.size = 0.1778;  % 17.78 cm = 7 inches (CORRECTED - user verified actual hardware)
    config.apriltag.base.id = 0;
    config.apriltag.base.mounting = 'rover_top_center';
    config.apriltag.base.physicalRotation = 180;  % degrees - tag rotated 180° to align yaw

    % Notes on tag orientation:
    % - Tag has been physically rotated 180° on the rover
    % - This aligns yaw=0° with rover forward direction (+X)
    % - Tag's X-axis (red) points toward corner 1 (with small white square)
    % - After rotation: Tag X-axis → Rover forward direction

    % End effector tag (optional - for arm tracking)
    config.apriltag.endEffector = struct();
    config.apriltag.endEffector.family = 'tag36h11';
    config.apriltag.endEffector.size = 0.11;  % 11 cm (ACTUAL - user installed on gripper)
    config.apriltag.endEffector.id = 2;  % Tag ID 2 on gripper (user confirmed)
    config.apriltag.endEffector.mounting = 'gripper';
    config.apriltag.endEffector.notes = 'Use sensor fusion with forward kinematics when occluded';

    % Additional rover tags (for occlusion handling)
    config.apriltag.additionalTags = struct();
    config.apriltag.additionalTags.recommended = true;
    config.apriltag.additionalTags.locations = {'rear_center', 'left_side', 'right_side'};
    config.apriltag.additionalTags.ids = [2, 3, 4];
    config.apriltag.additionalTags.purpose = 'Redundancy when arm occludes primary tag';

    %% Workspace Dimensions (VALIDATED)
    config.workspace = struct();

    % Ground plane workspace (tested area)
    config.workspace.xRange = [-3.0, 3.0];  % meters (left to right)
    config.workspace.yRange = [-2.0, 2.0];  % meters (back to forward)
    config.workspace.zRange = [0.0, 1.5];   % meters (ground to max height)

    % Tested positions during validation
    config.workspace.testedPositions = {
        'Center: (0, 0)';
        'Right Edge: (+2m, 0)';
        'Left Edge: (-2m, 0)';
        'Front Edge: (0, +1.5m)';
        'Back Edge: (0, -1.5m)';
        'Front-Right: (+2m, +1.5m)';
        'Front-Left: (-2m, +1.5m)';
        'Back-Right: (+2m, -1.5m)';
        'Back-Left: (-2m, -1.5m)';
    };

    % Accuracy characteristics
    config.workspace.positionAccuracy = struct();
    config.workspace.positionAccuracy.center = 50;      % mm (excellent)
    config.workspace.positionAccuracy.edges = 100;      % mm (good)
    config.workspace.positionAccuracy.typical = 70;     % mm (average)

    config.workspace.rotationAccuracy = struct();
    config.workspace.rotationAccuracy.yaw = 3;          % degrees (typical)
    config.workspace.rotationAccuracy.range = [2, 5];   % degrees (min-max)

    %% Coordinate Systems (DEFINITION)
    config.coordinates = struct();

    % World frame (global/fixed)
    config.coordinates.world = struct();
    config.coordinates.world.origin = 'Ground plane below camera';
    config.coordinates.world.xAxis = 'Right (+X), Left (-X)';
    config.coordinates.world.yAxis = 'Forward (+Y), Back (-Y)';
    config.coordinates.world.zAxis = 'Up (+Z), Down (-Z)';
    config.coordinates.world.handedness = 'right-handed';

    % Camera frame
    config.coordinates.camera = struct();
    config.coordinates.camera.origin = 'Camera optical center (4m above ground)';
    config.coordinates.camera.xAxis = 'Right (+X) - same as world';
    config.coordinates.camera.yAxis = 'Down (+Y) - opposite of world Y';
    config.coordinates.camera.zAxis = 'Forward/Depth (+Z) - opposite of world Z';
    config.coordinates.camera.handedness = 'right-handed';

    % Rover/Tag frame (after physical 180° rotation)
    config.coordinates.tag = struct();
    config.coordinates.tag.origin = 'Tag center';
    config.coordinates.tag.xAxis = 'Rover forward (red arrow to corner 1)';
    config.coordinates.tag.yAxis = 'Rover left (green arrow to corner 2)';
    config.coordinates.tag.zAxis = 'Perpendicular to tag (points down into ground)';
    config.coordinates.tag.handedness = 'right-handed';
    config.coordinates.tag.notes = 'Tag physically rotated 180° to align with rover';

    % Transformation matrices
    config.coordinates.transforms = struct();
    config.coordinates.transforms.cameraToWorld = [1,  0,  0;
                                                   0, -1,  0;
                                                   0,  0, -1];
    config.coordinates.transforms.notes = 'Rotation matrix to transform from camera frame to world frame';

    %% Validated Performance (TEST RESULTS)
    config.performance = struct();

    % Distance measurement test results
    config.performance.distance = struct();
    config.performance.distance.testDate = '2025-10-29';
    config.performance.distance.expectedDistance = 1.0;  % meters
    config.performance.distance.typicalError = 0.070;    % meters (70mm)
    config.performance.distance.errorRange = [0.050, 0.100];  % meters

    % Rotation measurement test results
    config.performance.rotation = struct();
    config.performance.rotation.testDate = '2025-10-29';
    config.performance.rotation.expectedRotation = 90;   % degrees
    config.performance.rotation.typicalError = 3;        % degrees
    config.performance.rotation.errorRange = [2, 5];     % degrees

    % Detection characteristics
    config.performance.detection = struct();
    config.performance.detection.frameRate = 30;         % Hz (approximate)
    config.performance.detection.maxRange = 6.0;         % meters (estimated)
    config.performance.detection.minTagSize = 0.05;      % meters (5cm at 2-3m distance)
    config.performance.detection.motionBlurSensitive = true;

    %% Calibration Notes (HISTORY)
    config.calibration = struct();

    config.calibration.history = {
        'Initial calibration: fx=800, fy=959.63 with distortion [0.05, -0.05]';
        'Issue: Large Z-errors (400-500mm) at left/right edges';
        'Solution: Switched to ORIGINAL parameters (fx=1050, fy=1050, no distortion)';
        'Result: Much better accuracy - 50-100mm position, 3-5° rotation';
        'Physical modification: Rotated AprilTag 180° to fix yaw alignment';
        'Validation date: 2025-10-29';
    };

    config.calibration.status = 'VALIDATED';
    config.calibration.recommendations = {
        'Keep using ORIGINAL camera parameters';
        'Do not apply distortion correction';
        'Ensure AprilTag remains physically rotated 180°';
        'Consider adding multiple tags for occlusion handling';
        'Implement sensor fusion with wheel odometry for robustness';
    };

    %% Usage Examples
    config.examples = struct();

    config.examples.initializeCamera = [...
        'kinectObj = imaq.VideoDevice(config.camera.deviceName, config.camera.deviceID, config.camera.colorFormat);', ...
        'kinectObj.ReturnedColorSpace = config.camera.returnedColorSpace;'];

    config.examples.detectTag = [...
        '[ids, locs, poses] = readAprilTag(grayFrame, config.apriltag.base.family, config.camera.intrinsics, config.apriltag.base.size);'];

    config.examples.transformToWorld = [...
        'worldPos = transformToWorld(poses(1).Translation, config.camera.position);', ...
        'worldRot = transformRotationToWorld(poses(1).Rotation);', ...
        'eulerAngles = rotm2eul(worldRot, ''XYZ'') * 180/pi;'];

    %% Helper Functions Reference
    config.helperFunctions = struct();
    config.helperFunctions.transformToWorld = 'worldPos = [cameraPos(1), -cameraPos(2), kinectPos(3) - cameraPos(3)]';
    config.helperFunctions.transformRotationToWorld = 'worldRot = R_cam_to_world * cameraRot; where R_cam_to_world = [1,0,0; 0,-1,0; 0,0,-1]';
    config.helperFunctions.angleNormalization = 'angle = mod(angle + 180, 360) - 180;  % Normalize to [-180, 180]';

    %% System Status
    config.status = struct();
    config.status.validated = true;
    config.status.lastValidated = '2025-10-29';
    config.status.validator = 'User testing with distance and rotation tests';
    config.status.readyForDeployment = true;
    config.status.knownIssues = {
        'Arm occlusion of base tag - recommend adding multiple tags';
        'End effector tracking not yet implemented - recommend sensor fusion';
        'Z-axis of tag points downward (cosmetic only, does not affect navigation)';
    };

    config.status.nextSteps = {
        'Add additional AprilTags (ID 2, 3, 4) for occlusion robustness';
        'Implement Kalman filter for sensor fusion (AprilTag + Odometry)';
        'Test end effector tracking with 5cm tag';
        'Integrate with rover navigation controller';
    };

    %% Display summary if called directly
    if nargout == 0
        fprintf('==============================================\n');
        fprintf('Vision System Configuration Summary\n');
        fprintf('==============================================\n\n');
        fprintf('Camera: %s\n', config.camera.model);
        fprintf('  Resolution: %dx%d\n', config.camera.imageSize(2), config.camera.imageSize(1));
        fprintf('  Focal Length: fx=%.1f, fy=%.1f\n', config.camera.focalLength(1), config.camera.focalLength(2));
        fprintf('  Height: %.1fm\n', config.camera.height);
        fprintf('\nAprilTag: %s\n', config.apriltag.base.family);
        fprintf('  Size: %.1fcm (%.3fm)\n', config.apriltag.base.size*100, config.apriltag.base.size);
        fprintf('  Base Tag ID: %d\n', config.apriltag.base.id);
        fprintf('  Physical Rotation: %d°\n', config.apriltag.base.physicalRotation);
        fprintf('\nWorkspace:\n');
        fprintf('  X: [%.1f, %.1f] m\n', config.workspace.xRange(1), config.workspace.xRange(2));
        fprintf('  Y: [%.1f, %.1f] m\n', config.workspace.yRange(1), config.workspace.yRange(2));
        fprintf('  Z: [%.1f, %.1f] m\n', config.workspace.zRange(1), config.workspace.zRange(2));
        fprintf('\nAccuracy (Validated):\n');
        fprintf('  Position: ~%dmm typical, %d-%dmm range\n', ...
               config.workspace.positionAccuracy.typical, ...
               config.workspace.positionAccuracy.center, ...
               config.workspace.positionAccuracy.edges);
        fprintf('  Rotation: ~%d° typical, %d-%d° range\n', ...
               config.workspace.rotationAccuracy.yaw, ...
               config.workspace.rotationAccuracy.range(1), ...
               config.workspace.rotationAccuracy.range(2));
        fprintf('\nStatus: %s\n', config.calibration.status);
        fprintf('Last Validated: %s\n', config.status.lastValidated);
        fprintf('Ready for Deployment: %s\n', mat2str(config.status.readyForDeployment));
        fprintf('==============================================\n');
    end
end
