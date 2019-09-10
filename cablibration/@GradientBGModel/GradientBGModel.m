classdef GradientBGModel < matlab.System
    properties (Access = private)
        G;
        H;
        imsz;
        C;
    end
    properties (SetAccess = private)
        gMag;
        gDir;
        mask;
    end
    properties (Nontunable)
        LearningRate = 0.05;
        Orientations = 6;
        BinSize = 2;
    end
    methods
        function obj = GradientBGModel(varargin)
            setProperties(obj, numel(varargin), varargin{:});
        end
        function set.LearningRate(obj, l)
            if ~isscalar(l) || l < 0 || l > 1
                error('Learning rate must be scalar from range <0,1>');
            end
            obj.LearningRate = l;
        end
    end
    methods (Access = protected)
        function setupImpl(obj)
            obj.H = single(fspecial('gaussian',7,1));
            obj.C = vision.Convolver('OutputSize', 'Same as first input');
        end
        function releaseImpl(obj)
            release(obj.C);
        end
        function [mask,mag,grad] = stepImpl(obj, im)
            im = step(obj.C, im, obj.H);
            [obj.gMag,obj.gDir] = gradientMag(im,0,9,0.02,1);
            gh = gradientHist(obj.gMag,obj.gDir,obj.BinSize,obj.Orientations,2);
            if isempty(obj.G)
                obj.G = gh;
            else
                obj.G = (1-obj.LearningRate)*obj.G + obj.LearningRate*gh;
            end
            obj.mask = max(abs(obj.G - gh), [], 3);
            [h,w,~] = size(im);
            obj.mask = imresize(obj.mask,[h,w],'nearest');
            mask = obj.mask;
            mag = obj.gMag;
            grad = obj.gDir;
        end
    end
end