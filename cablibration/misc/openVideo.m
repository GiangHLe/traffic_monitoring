function hsrc = openVideo(video, varargin)

if exist(video,'file') == 2
    % File - assume a media file accepted by file reader
    hsrc = vision.VideoFileReader(video,varargin{:});
    return;
end

[path,~,~] = fileparts(video);
if exist(video,'dir') || exist(path,'dir')
    % Directory or directory with file mask
    hsrc = ImageSequenceReader(video,varargin{:});
    return;
end

error('Cannot open media %s', video);