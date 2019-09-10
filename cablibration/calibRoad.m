function [C, P, im] = calibRoad(video, varargin)
% Get camera parametrs from road video.
%
%   C = calibRoad(video, ...);
%   C = calibRoad(video, 'Param1', 'Value1', ...);
%   [C,im] = calibRoad(...);
%
% Get camera parametrs from video. The video parameter can be either a
% multimedia file (avi, mp4, etc.) or a directory with image sequence.
% Accepted options are:
%
% Show         [false] Whether to visualize current calibration state
% StartAt      [1] Starting frame
% FinishAt     [inf] Last frame
% FrameStep    [4] Time tep in frames
% FocalRange   [] 1x2 vector - prior for focal length in pixels
% HorizonRange [] 1x2 vector - prior for horizon inclination in degrees
% ROI          [] Polygon of active area
% pp           [] 1x2 vector - position of principal point
%
% See also RoadCalibration

%% Init user values
dfs = { ...
    'Show', false, ...
    'StartAt', 1, 'FinishAt', inf, 'FrameStep',4, ...
    'FocalRange', [], 'HorizonRange', []...
    'ROI', [],...
    'pp', [] ...
    };

[show,first_frame,last_frame,frame_step,focal_range,horizon_range,ROI,pp] = getPrmDflt(varargin, dfs, 1);

%% Init objects
if show
    hsnk = vision.VideoPlayer;
end

if ischar(video)
    hsrc = openVideo(video,'ImageColorSpace','Intensity','VideoOutputDataType','single');
else
    hsrc = video;
end

%% Init calibrator
C = RoadCalibration;
C.focalRange = focal_range;
C.horizonRange = horizon_range;
C.pp = pp;
C.ROI = ROI;

%% Main loop
frameId = 0;
while ~isDone(hsrc)
    frame  = hsrc.step;
    frameId = frameId + 1;
    if frameId < first_frame, continue; end;
    if frameId > last_frame, break; end;
    if mod(frameId,frame_step) ~= 0, continue; end;
    C.step(frame);
    if show
        hsnk.step(C.show(frame));
    end
end

release(hsrc);
if show
    release(hsnk);
end

%% Finish

if nargout==3, im = C.show(frame); end;
P = C.roadPlane;
C = C.toStruct;

