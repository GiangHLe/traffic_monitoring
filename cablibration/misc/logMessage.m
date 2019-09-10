function logMessage(format, varargin)
d = datestr(datetime,'yyyy-mm-dd HH:MM:SS.FFF');
fprintf(['[',d,']: ',format,'\n'], varargin{:});