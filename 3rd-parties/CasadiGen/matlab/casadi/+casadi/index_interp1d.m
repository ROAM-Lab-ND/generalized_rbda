function varargout = index_interp1d(varargin)
    %INDEX_INTERP1D [INTERNAL] 
    %
    %  double = INDEX_INTERP1D([double] x, double xq, bool equidistant)
    %
    %
  [varargout{1:nargout}] = casadiMEX(275, varargin{:});
end
