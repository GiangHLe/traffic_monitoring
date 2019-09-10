classdef DiamondSpace < handle
    % DiamondSpace is a point-point transformation of real projective plane.
    %
    % DiamondSpace Methods:
    %
    % accumulateLines  - Transform lines from cartesian space to DS and accumulate them
    % findMaximum      - Transfrom a point from DS to cartesian space
    % backproject      - Mapping from Diamond Space to Cartesian Space
    % project          - Mapping from Cartesian Space to Diamond Space 
    % emptySpace       - Empty accumulator and its mask
    %
    % DiamondSpace Properties:
    %
    % spaceSize        - Size of accumulator
    % spaceMask        - Mask of valid bins in ds
    % spaceBins        - List of Diamond Space points present in ds
    % cartesianBins    - List of Cartesian points in ds
    % ds               - Accumulator
    % u                - Look-up table for normalization of ds bins to
    %                    Diamond Space
    %
    % Example:
    %
    %     % Generate points
    %     N = 1000;
    %     c = 10 * (rand(N,2)-0.5);
    %     % Project c to Diamond space
    %     p = DiamondSpace.project(c);
    %     % Transform back
    %     c1 = DiamondSpace.backproject(p);
    %     
    %     subplot(1,2,1);
    %     plot(c(:,1),c(:,2),'+',c1(:,1),c1(:,2),'o');
    %     legend({'c','c1'}); title('Cartesian Space');
    %     subplot(1,2,2);
    %     plot(p(:,1),p(:,2),'+');
    %     legend('p'); title('Diamond Space');
    %
    % For more details see:
    %  Dubska et al., Real Projective Plane Mapping for Detection of
    %  Orthogonal Vanishing Points, BMVC 2013
    %
    % Copyright (c) 2016, Roman Juranek <ijuranek@fit.vutbr.cz>
    %
    
    %%
    properties (Dependent)
        % Accumulator size.
        spaceSize;
    end
    %% Private properties
    % These should be private, but they are sometimes usefull so I keep
    % them visible.
    properties (SetAccess = private)
        % Mask of bins (pixels) in ds that are valid.
        spaceMask
        % Mapping of bins (pixels) in ds to Diamond Space.
        spaceBins;
        % Mapping of bins (pixels) in ds to Cartesian space.
        cartesianBins;
        % Diamond Space accumulator. NaN values marks bins that are not
        % valid,
        ds;
        % Lookup table for mapping od pixels in the accumulator to Diamond
        % Space
        u;
    end
    %% Methods
    methods
        function obj = DiamondSpace(sz)
            % Init DiamondSpace object with accumulator size sz.
            if nargin==0, sz=256; end;
            if ~isscalar(sz)
                error('DiamondSpace:InvalidSize','Size must be a scalar value');
            end
            sz = sz + (1-mod(sz,2)); % Force sz to be odd value
            [obj.ds,obj.spaceMask] = obj.emptySpace(sz);
            obj.u = linspace(-1,1,obj.spaceSize);
            [r,c] = find(obj.spaceMask);
            obj.spaceBins = obj.u([c,r]);
            obj.cartesianBins = obj.backproject(obj.spaceBins);
        end
        function sz = get.spaceSize(obj)
            sz = size(obj.ds,1);
        end
        function accumulateLines(obj, lines)
            % Transform lines and accumulate them to the space
            if ~isfloat(lines) || ~ismatrix(lines) || size(lines,1)~=4
                error('DiamondSpace:invalidLines','Lines must be a 4xN floating point matrix');
            end
            if ~isa(lines,'single'), lines = single(lines); end;
            obj.ds = obj.ds + double(mx_raster_space(size(obj.ds,1), lines));
        end
        function p = findMaximum(obj, mask)
            % Get position of maximum in the accumulator.
            if nargin==2, d = obj.ds .* mask;
            else d = obj.ds; end;
            [r,c] = find(d==max(d(:)),1,'first');
            p = obj.u([c,r]);
        end
        function clear(obj)
            [obj.ds,obj.spaceMask] = obj.emptySpace(obj.spaceSize);
        end
    end
    %% Static methods
    methods (Static)
        function c = backproject(p)
            % c = backproject(p) returns carresian coordinates c of Diamond
            % Space points p, where p is asumed to be Nx2 vector with N
            % points.
            assert(size(p,2)==2);
            p = num2cell(p,1); [u,v] = deal(p{:});
            c = [v, sign(v).*v + sign(u).*u - 1, u];
            eps = 1e-6; reg = abs(c(:,3)) > eps;
            c( reg,:) = bsxfun(@rdivide, c(reg,:), c(reg,3));
            c(~reg,:) = normr(c(~reg,:)); c(~reg,3) = 0;
        end
        function p = project(c)
            % p = project(c) returns coordinates of Cartesian points c in
            % Diamond space. c is assumed to be Nx2 matrix with cartesian
            % points, or Nx3 with homogenous points.
            assert(any(size(c,2)==[2,3])); n = size(c,1);
            if size(c,2)==2, c = [c, ones(n,1)]; end;
            c = num2cell(c,1);
            [x,y,w] = deal(c{:});
            p = [-w, -x, sign(x.*y).*x + y + sign(y).*w];
            p = bsxfun(@rdivide, p(:,1:2), p(:,3));
        end
        function [space,mask] = emptySpace(sz)
            % [space,mask] = emptySpace(sz) returns an empty Diamond Space
            % where NaN values mark unused bins not belonging to the space,
            % valud bins are filled with zeros.
            space = nan(sz);
            k = ceil(sz/2);
            p = [k,0.5; sz+0.5,k; k,sz+0.5; 0.5,k];
            mask = poly2mask(p(:,1), p(:,2), sz, sz);
            space(mask) = 0;
        end
    end
end
