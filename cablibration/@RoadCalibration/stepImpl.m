function stepImpl(obj, frame)
%% Init and time increment
if obj.numFrames == 0, init; end;
obj.numFrames = obj.numFrames+1;

%% Update background model;
[edgeMask,edgeMag,edgeGrad] = step(obj.B, frame);

%% Get support for VPs and update them
[trackPts, oldPts] = trackPoints;
updateVP1;
updateVP2;
updateVP3AndFocal;

%% Update the set of points
getNewPointsForTracking;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Init for first call of step method
    function init
        [h,w,~] = size(frame);
        obj.frameSize = [w,h];
        obj.pts = single(step(obj.pointDetector, frame));
        initialize(obj.tracker, obj.pts, frame);
        if isempty(obj.pp), obj.pp = obj.frameSize/2; end;
        obj.frameOrigin = obj.pp;
        obj.frameNormalizer = obj.pp(1)*2;
        obj.Normalizer = @(p,t)geometry.normalizePoints(p,obj.frameOrigin,obj.frameNormalizer,t);
        
        %obj.Normalize = @(p,t)geometry.normalizePoints(p,obj.frameOrigin,obj.frameNormalizer,0);
        %obj.Denormalize = @(p,t)geometry.normalizePoints(p,obj.frameOrigin,obj.frameNormalizer,1);
        
        obj.B.step(frame);
    end

%% Get new and old positions of tracked points
    function [trackPts,oldPts] = trackPoints
        % Track points and select valid ones
        [trackPts,validPts] = step(obj.tracker, frame); % Track points
        movingPts = geometry.pointToPointDist(trackPts, obj.pts) > obj.pointMoveThreshold; % Select moving points
        validPts = validPts & movingPts;
        trackPts = trackPts(validPts,:); oldPts = obj.pts(validPts,:);
    end

%% VP1 update from point motion
    function updateVP1
        lines1 = geometry.linesFromEndpoints(obj.Normalizer(oldPts,0), obj.Normalizer(trackPts,0));
        obj.lines{1,obj.numFrames} = lines1;
        % If there were lines (moving points)...
        if size(lines1,2) > 0
            % Accumulate lines to vpAcc1 and update first vanishing point
            oldVp1 = obj.vp{1};
            obj.vpAcc(1).accumulateLines(lines1);
            V1 = obj.vpAcc(1).findMaximum; % [u,v] normalized maxima position in accumulator space
            obj.vp{1} = obj.Normalizer(DiamondSpace.backproject(V1),1);
            % Update search masks for VP2 if necessary
            if geometry.pointToPointDist(oldVp1(1:2),obj.vp{1}(1:2)) > 1
                obj.lastVp1Update = obj.numFrames;
                obj.gradientMask = getVP1DirectionVectors;
                obj.focalMask = getFocalMask;
                obj.horizonMask = getHorizonMask;
            end
        end
    end

%% VP2 update from foreground edges
    function updateVP2
        if obj.isVp1Stable
            % Get edges that could coincide with VP2
            edgeMask = edgeMask > obj.BGSensitivity;
            [ePts,eMag,eGrad] = obj.getVP2EdgePixels(frame,edgeMask,edgeMag,edgeGrad);
            % Filter hypotheses using ROI
            if ~isempty(obj.ROI)
                ePtsIn = inpolygon(ePts(:,1),ePts(:,2),obj.ROI(:,1),obj.ROI(:,2));
                ePts(~ePtsIn,:) = [];
                eMag(~ePtsIn,:) = [];
                eGrad(~ePtsIn,:) = [];
            end
            ePts = obj.Normalizer(ePts,0);
            % Transform edges to lines
            lines2 = geometry.linesFromEdges(ePts,eMag,eGrad);           
            obj.lines{2,obj.numFrames} = lines2;
            if size(lines2,2) > 0
                oldVp2 = obj.vp{2};
                % Accumulate the lines
                obj.vpAcc(2).accumulateLines(lines2);
                % Search for maximum and apply mask
                V2 = obj.vpAcc(2).findMaximum(obj.vp2SearchMask);
                % Get the VP2 position
                obj.vp{2} = obj.Normalizer(DiamondSpace.backproject(V2),1);
                if geometry.pointToPointDist(oldVp2(1:2),obj.vp{2}(1:2)) > 1
                    obj.lastVp2Update = obj.numFrames;
                end
            end
        end
    end

%% VP3 and focal is calculated automatically
    function updateVP3AndFocal
        if obj.lastVp1Update==obj.numFrames || obj.lastVp2Update==obj.numFrames
            if (obj.vp{1}(3) == 0 || obj.vp{2}(3) == 0)
                obj.vp{3} = [0,-1,0];
            else
                V3 = cross([obj.vp{1}(1:2),obj.focal] - [obj.pp,0],[obj.vp{2}(1:2),obj.focal] - [obj.pp,0]);
                V3 = V3 ./ V3(3) .* obj.focal;
                obj.vp{3} = [V3(1:2) + obj.pp,1];
            end
        end
    end

%% Update the set of points for tracking
    function getNewPointsForTracking
        if obj.numFrames - obj.lastPtsUpdate >= obj.updatePtsEvery
            newPts = single(step(obj.pointDetector, frame));
            validNewPts = geometry.uniquePoints(trackPts, newPts, obj.pointLocationTolerance);
            obj.pts = [trackPts; validNewPts];
            if ~isempty(obj.ROI)
                ptsIn = inpolygon(obj.pts(:,1),obj.pts(:,2),obj.ROI(:,1),obj.ROI(:,2));
                obj.pts(~ptsIn,:) = [];
            end
            if ~isempty(obj.pts)
                obj.tracker.setPoints(obj.pts);
                obj.lastPtsUpdate = obj.numFrames;
            end
        end
    end

%%
    function mask = getFocalMask
        m = obj.vpAcc(2).spaceMask;
        mask = zeros(size(m));
        range = obj.focalRange;
        if isempty(range)
            mask(m) = true;
        else
            pp = obj.pp(1:2);
            vp1 = obj.vp{1}(1:2) - pp;
            vp2 = obj.Normalizer(obj.vpAcc(2).cartesianBins,1);
            vp2 = bsxfun(@minus, vp2(:,1:2), pp);
            f = real(sqrt(-(vp1)*(vp2)'));
            mask(m) = (f > range(1)) & (f < range(2));
        end
    end


%%
    function mask = getHorizonMask
        m = obj.vpAcc(2).spaceMask;
        mask = zeros(size(m));
        range = obj.horizonRange;
        if isempty(range)
            mask(m) = true;
        else
            x = obj.vpAcc(2).cartesianBins;  n = size(x,1);
            vp = obj.Normalizer(obj.vp{1},0);
            horLine = cross(x, repmat(vp,n,1));
            h = normr(horLine(:,1:2)); % normal vector
            a = atan2d(h(:,1),h(:,2));
            a(a>90) = a(a>90) - 180;
            a(a<-90) = a(a<-90) + 180;
            v = a>range(1) & a<range(2);
            mask(m) = v;
        end
    end

%%
    function d = getVP1DirectionVectors
        [x,y] = meshgrid(1:obj.frameSize(1), 1:obj.frameSize(2));
        p = obj.vp{1};
        if p(3) < 0.005
            u = repmat(p(1), size(x));
            v = repmat(p(2), size(x));
        else
            u = x - p(1);
            v = y - p(2);
        end
        a = -atan2(v,u);
        d = cat(3,sin(a),cos(a));
    end

end % stepImpl