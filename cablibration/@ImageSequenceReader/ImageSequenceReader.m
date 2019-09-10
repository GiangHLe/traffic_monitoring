classdef ImageSequenceReader < matlab.System & matlab.system.mixin.FiniteSource
    % Imgae sequences as video
    
    %%
    properties (Access=private,Nontunable)
        fs;
    end
    
    %%
    properties (Access=private,PositiveInteger)
        j = 1;
    end
    
    %%
    properties (Access=private)
        DataConvert;
        ColorConvert;
        FrameSize;
        BasePath;
        FileMask;
    end
    
    %%
    properties (Nontunable)
        ImageColorSpace = 'RGB';
        VideoOutputDataType = 'Single';
        Path;
        FrameStep = 1;
    end
    
    %%
    methods
        function obj = ImageSequenceReader(path, varargin)
            setProperties(obj, numel(varargin), varargin{:});
            obj.Path = path;
            if exist(obj.Path,'dir')
                obj.BasePath = obj.Path;
                obj.FileMask = '*';
            else
                [obj.BasePath,mask,ext]= fileparts(obj.Path);
                obj.FileMask = [mask, ext];
            end
            obj.getFileList();
            if ~isempty(obj.fs)
                im = obj.readFrame(1);
                obj.FrameSize = size(im);
            else
                warning('ImageSequenceReader:emptySequence','Empty video sequence');
            end
        end
    end
    
    %%
    methods (Access = protected)
        function setupImpl(obj)
            obj.j = 1;
            obj.DataConvert = vision.ImageDataTypeConverter('OutputDataType',obj.VideoOutputDataType);
            obj.ColorConvert = vision.ColorSpaceConverter('Conversion', 'RGB to intensity');
        end
        function im = stepImpl(obj)
            im = obj.readFrame(obj.j);
            if any(size(im) ~= obj.FrameSize)
                error('ImageSequenceReader:videoSizeChanged','Size of video frames must be constant through sequence.');
            end
            im = obj.ColorConvert.step(im);
            im = obj.DataConvert.step(im);
            obj.j = obj.j+obj.FrameStep;
        end
        function i = infoImpl(obj)
            i = struct( ...
                'VideoSize', obj.FrameSize([2,1]), ...
                'NumberOfFrames', numel(obj.fs)...
                );
        end
        function b = isDoneImpl(obj)
            b = obj.j > numel(obj.fs);
        end
        function resetImpl(obj)
            obj.j = 1;
        end
        function releaseImpl(obj)
            release(obj.ColorConvert);
            release(obj.DataConvert);
        end
    end
    methods (Access=private)
        function im = readFrame(obj, i)
            im = imread(obj.fs{i});
        end
        function getFileList(obj)
            files = dir(fullfile(obj.BasePath,obj.FileMask));
            files([files.isdir]) = [];
            obj.fs = cell(1,numel(files));
            for i = 1:numel(obj.fs)
                obj.fs{i} = fullfile(obj.BasePath, files(i).name);
            end
        end
    end
end