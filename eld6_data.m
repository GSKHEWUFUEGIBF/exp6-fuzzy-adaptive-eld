function sys = eld6_data(PD)
%ELD6_DATA  IEEE 6-unit thermal test system (Z.-L. Gaing, IEEE Trans. Power Syst., 2003;
%   also Example 7.8 in H. Saadat, "Power System Analysis").
%
%   Fuel cost of unit i :  F_i(P_i) = a_i + b_i*P_i + c_i*P_i^2           ($/h, P in MW)
%   Transmission loss   :  P_L = P*B*P' + B0*P' + B00                       (MW)
%   The loss coefficients are published in per unit on a 100 MVA base; they are
%   converted here so that P is in MW:  B = Bpu/100,  B0 = B0pu,  B00 = 100*B00pu.
%
%   sys = eld6_data()      load demand 1263 MW
%   sys = eld6_data(PD)    any other load demand (MW)

if nargin < 1, PD = 1263; end
sys.name = 'IEEE 6-unit thermal system';
sys.n    = 6;
sys.PD   = PD;                                     % load demand (MW)

sys.a    = [240 200 220 200 220 190];              % $/h
sys.b    = [7.0 10.0 8.5 11.0 10.5 12.0];          % $/MWh
sys.c    = [0.0070 0.0095 0.0090 0.0090 0.0080 0.0075];   % $/MW^2 h

sys.Pmin = [100  50  80  50  50  50];              % MW
sys.Pmax = [500 200 300 150 200 120];              % MW

% ramp-rate limits and prohibited operating zones (Case 2 only)
sys.P0   = [440 170 200 150 190 110];              % output in the previous hour (MW)
sys.UR   = [ 80  50  65  50  50  50];              % up-ramp limit (MW/h)
sys.DR   = [120  90 100  90  90  90];              % down-ramp limit (MW/h)
sys.POZ  = [210 240 350 380                        % [zone1_lo zone1_hi zone2_lo zone2_hi] (MW)
             90 110 140 160
            150 170 210 240
             80  90 110 120
             90 110 140 150
             75  85 100 105];

% B-loss coefficients, per unit on 100 MVA (as published)
sys.Bpu = [ 0.0017  0.0012  0.0007 -0.0001 -0.0005 -0.0002
            0.0012  0.0014  0.0009  0.0001 -0.0006 -0.0001
            0.0007  0.0009  0.0031  0.0000 -0.0010 -0.0006
           -0.0001  0.0001  0.0000  0.0024 -0.0006 -0.0008
           -0.0005 -0.0006 -0.0010 -0.0006  0.0129 -0.0002
           -0.0002 -0.0001 -0.0006 -0.0008 -0.0002  0.0150 ];
sys.B0pu  = 1e-3*[-0.3908 -0.1297 0.7047 0.0591 0.2161 -0.6635];
sys.B00pu = 0.0056;
sys.baseMVA = 100;

% the same coefficients with P in MW
sys.B   = sys.Bpu/sys.baseMVA;      % 1/MW
sys.B0  = sys.B0pu;                 % -
sys.B00 = sys.B00pu*sys.baseMVA;    % MW
end
