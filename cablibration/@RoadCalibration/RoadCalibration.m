classdef RoadCalibration < matlab.System
    % RoadCalibration is an algorithm for calibration of static cameras capturing straight roads.
    %
    % This class implements the algorithm described in [1]. It uses modified
    % piece-wise linear Hough Transform, called Diamond Space, to estimate
    % two main vanishing points (VP). VP1 in the direction of car movement,
    % and VP2 prependicular to VP1 in the road plane. From the VPs, camera
    % focal length and camera orientation in the scene can be recovered.
    %
    % The assumptions for successful calibration are following. 1/
    % sufficient number of frames; 2/ VP2 is outside of image plane and not
    % in infinity; 3/ camera is static; 4/ The road in the region of
    % interest is straight.
    %
    % IMPORTANT NOTE:
    % RoadCalibration objects cannot be saved in .mat files (yet).
    %
    % RoadCalibration methods:
    % RoadCalibration - Create the object
    % step            - Process a frame and update calibration information
    % show            - Visualize current state of calibration
    %
    % RoadCalibration properties that can be modified:
    % horizonRange - Range of possible horizon inclinations
    % focalRange   - Range of possible focal lengths
    % pp           - Principal point of camera
    % frameMask    - Region of interest
    %
    % RoadCalibration properties visible to user:
    % vp           - Vanishing points
    % horizon      - Horizon line
    % focal        - Estimated focal lengths
    % frameSize    - Input frame size [width, height]
    % numFrames    - Number of processed frames
    %
    % Requirements:
    % * Computer Vision System Toolbox
    % * DiamondSpace
    % * GradientBGModel
    % * Piotr's Toolbox
    %
    % References:
    % [1] Dubská et al.; Fully Automatic Roadside Camera Calibration for
    % Traffic Surveillance; IEEE Transactions on Intelligent Transportation
    % Systems; 2014 http://ieeexplore.ieee.org/stamp/stamp.jsp?arnumber=6909022
    %
    % Copyright (c) 2016, Roman Juranek <ijuranek@fit.vutbr.cz>

    %% Object properties
    properties
        % Principal point of camera. We set this to the center. If
        % you know the correct position, you can set it by yourself. But
        % the calibration and measurements are not sensivite to this value
        pp = [0,0];
        % Scene scale. This is set to 1. You need to set this value in
        % order to make measurements in the scene.
        scale = 1;
    end
    %% Algorithm properties
    properties (Nontunable)
        % Range of possible horizon inclination in degrees.
        % E.g. horizonRange = [-5,5]; You can narrow search space for
        % horizon to make calibration more robust.
        horizonRange = [];
        % Range of possible focal ranges in pixels. If you know, at least
        % approximately, field of view of your camera, you can narrow
        % search space focal length. E.g. focalRange = [600,800]; Set it
        % to empty vector for full search.
        focalRange = [];
        % Size of accumulator space
        accumulatorSize = 512;
        % Properties for point tracker
        TrackerProps = {'MaxBidirectionalError',1,'BlockSize',[11,11]};
        % Properties for point detector
        PointDetectorProps = {'Method','Minimum eigenvalue (Shi & Tomasi)'};
        % Properties of background model
        BGModelProps = {'LearningRate', 0.001};
        % Threshold for background model - low values will expose more
        % edges to accumulation of VP2.
        BGSensitivity = 0.35;
        % Number of frames for which VP1 must not move in order to start
        % calibration of VP2.
        vp1StabilityThreshold = 0;
        % Number of frames for which VP1 must not move in order to start
        % calibration of VP2.
        vp2StabilityThreshold = 5;
        % Number of frames after which points are updated
        updatePtsEvery = 8;
        % Min. length of movement vector to consider a point moving
        pointMoveThreshold = 8;
        % Radius (in pixels) to consider a point stable
        pointLocationTolerance = 2;
        % Region of interest polygon (e.g. from impoly)
        ROI;
    end
    %% Visible properties
    % These are visible to user because they represent algorithm results or
    % other useful information, but they can not be modified.
    properties (SetAccess = private)
        % Vanishing points. vp{1} is a point where movement of cars on road
        % converges; vp{2} is prependicular to vp{1} in road plane; vp{3}
        % is prependicular to road plane.
        vp = {[1,0,0],[0,1,0],[0,0,1]};
        % Size of input images.
        frameSize = [];
        % Number of processed frames
        numFrames = 0;
        % Lines that were used for estimation of vanishing points.
        % lines(1,:) for VP1, lines(2,:) for VP2.
        lines;
    end
    %% Private properties
    % Internal state of the algorithm
    properties (Access = private)
        % Accumulator spaces for the vanishing points
        vpAcc = DiamondSpace.empty;
        
        % Detecrtor of points to track
        pointDetector;
        % Point tracker
        tracker;
        % Points assigned to the tracker in the current frame
        pts;
        lastPtsUpdate = 0;
        
        % Image-space map with directions to VP1 for every pixel
        gradientMask;
        
        % Mask for vpAcc(2) marking possible locations of VP2 with respect to
        % frame principal point and VP1
        ppMask;
        % Mask for vpAcc(2) marking locations of VP2 that coincide tith
        % horizon prior with respect to VP1.
        horizonMask;
        % Mask for vpAcc(2) marking locations of VP2 that coincide tith
        % focal length prior with respect to VP1
        focalMask;
        
        % Last frame when VP1 was update
        lastVp1Update = 0;
        % Last frame when VP2 was update
        lastVp2Update = 0;
        
        % Origin of the image - center
        frameOrigin;
        % Normalization constant - half of frame with
        frameNormalizer;
        % Function for point normalization in cartesian space
        Normalizer;
        
        % Edge background model
        B;
        
        % Edge detector object
        edgeDetector;
    end
    
    %% Private properties that are calculated on demand
    properties (Access=private, Dependent)
        % isVP1Stable returns true if VP1 is stable for
        % vp1StabilityThreshold frames
        isVp1Stable;
        % isVP2Stable returns true if VP1 is stable for
        % vp2StabilityThreshold frames
        isVp2Stable;
        % Compound mask narrowing VP2 search area
        vp2SearchMask;
    end
    
    %%
    properties (SetAccess=private, Dependent)
        % Stability flag for the calibration
        isCalibrationStable;
        % Focal length of camera in pixels. This property is related to the
        % field of view of the camera - low values (compared to input image
        % size) marks wide lens, high values marks high zooms.
        focal;
        % Parameters of horizon line
        horizon;
        roadPlane;
    end
    
    %% Public methods
    methods (Access = protected)
        function setupImpl(obj)
            obj.tracker = vision.PointTracker(obj.TrackerProps{:});
            obj.pointDetector = vision.CornerDetector(obj.PointDetectorProps{:});
            obj.B = GradientBGModel(obj.BGModelProps{:});
            obj.vpAcc(1) = DiamondSpace(obj.accumulatorSize);
            obj.vpAcc(2) = DiamondSpace(obj.accumulatorSize);
            obj.edgeDetector = vision.EdgeDetector('Method','Canny');
        end
        
        function releaseImpl(obj)
            release(obj.pointDetector);
            release(obj.tracker);
            release(obj.B);
            release(obj.edgeDetector);
        end
        
        stepImpl(obj, frame);
    end
    %%
    methods
        function obj = RoadCalibration(varargin)
            obj@matlab.System;
            setProperties(obj, numel(varargin), varargin{:});
        end
        
        frame = show(obj, frame);
        
        function C = toStruct(obj)
            %C = struct('vp', {obj.vp}, 'pp', obj.pp, 'focal', obj.focal, 'horizon', obj.horizon, 'frameSize', obj.frameSize, 'scale', obj.scale);
            C = struct( ...
                'vp1', obj.vp{1}(1:2), 'vp2', obj.vp{2}(1:2), 'vp3', obj.vp{3}(1:2), ...
                'pp', obj.pp, 'focal', obj.focal);
        end
        
        function stable = get.isVp1Stable(obj)
            stable = (obj.lastVp1Update > 0) && ((obj.numFrames - obj.lastVp1Update) > obj.vp1StabilityThreshold);
        end
        
        function stable = get.isVp2Stable(obj)
            stable = (obj.lastVp2Update > 0) && ((obj.numFrames - obj.lastVp2Update) > obj.vp2StabilityThreshold);
        end
        
        function stable = get.isCalibrationStable(obj)
            stable = obj.isVp1Stable && obj.isVp2Stable;
        end
        
        function f = get.focal(obj)
            f = -1;
            if (obj.vp{1}(3) ~= 0 && obj.vp{2}(3) ~= 0)
                f = sqrt(-(obj.vp{1}(1:2) - obj.pp)*(obj.vp{2}(1:2) - obj.pp)');
            end;
        end
        
        function h = get.horizon(obj)
            h = [0,1,0];
            if (obj.vp{1}(3) ~= 0 && obj.vp{2}(3) ~= 0)
                h = cross(obj.vp{1}, obj.vp{2});
            end
        end
        
        function P = get.roadPlane(obj)
            w = [obj.vp{3}(1:2), obj.focal] - [obj.pp,0];
            P = [normr(w), 10]; % 10 is arbitrary choice
        end
        
        function set.focalRange(obj,range)
            if ~isempty(range) && (~isvector(range) || length(range)~=2 || range(2)<range(1))
                error('RoadCalibration:badFocalRange','Focal range must be two elemet vector');
            end
            obj.focalRange = range;
        end
        
        function mask = get.vp2SearchMask(obj)
            mask = obj.focalMask & obj.horizonMask;
        end
        
        function set.horizonRange(obj,range)
            if ~isempty(range) && (~isvector(range) || length(range)~=2 || range(2)<range(1))
                error('RoadCalibration:badHorizonRange','Horizon range must be two elemet vector');
            end
            obj.horizonRange = range;
        end
    end
    %% Static methods
    methods (Static)
        %%
        function wps = pointsToPlane(pts, focal, PP, P)
            n = size(pts, 1);
            pts = bsxfun(@minus, pts, PP); % [x-ppx, y-ppy]
            pts = [pts, repmat(focal,n,1)];  % [x-ppx, y-ppy, focal]
            t = dot(P,[PP,0,1])./(P(1:3) * pts');
            wps = bsxfun(@times, pts, t');
            wps = bsxfun(@plus, wps, [PP,0]);
        end
        
        %%
        function ips = pointsToScreen()
            ips = [];
        end
        
        %%
        function [C,P] = calibrationFromVP(vp1, vp2, pp)
            % recalc focal
            f = sqrt(-(vp1 - pp)*(vp2 - pp)');
            % recalc vp3
            V3 = cross([vp1,f]-[pp,0],[vp2,f]-[pp,0]);
            V3 = V3 ./ V3(3) .* f;
            vp3 = [V3(1:2) + pp,1];
            % recalc road plane
            w = [vp3(1:2),f] - [pp,0];
            P = [normr(w),10];
            % Compose calib info
            C.vp1 = vp1;
            C.vp2 = vp2;
            C.vp3 = vp3(1:2);
            C.pp = pp;
            C.focal = f;
        end
        
    end
end