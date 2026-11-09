function prm = eld_problem(sys, caseNo)
%ELD_PROBLEM  Optimisation settings for one test case.
%   Case 1 : generator limits + transmission loss            (convex, lambda-iteration applies)
%   Case 2 : Case 1 + ramp-rate limits + prohibited zones    (non-convex)
prm.a = sys.a;   prm.b = sys.b;   prm.c = sys.c;
prm.B = sys.B;   prm.B0 = sys.B0; prm.B00 = sys.B00;
prm.PD = sys.PD;
prm.POZ = sys.POZ;
if caseNo == 1
    prm.lo = sys.Pmin;   prm.hi = sys.Pmax;   prm.usePOZ = false;
else
    prm.lo = max(sys.Pmin, sys.P0 - sys.DR);  % ramp-rate limits shrink the operating range
    prm.hi = min(sys.Pmax, sys.P0 + sys.UR);
    prm.usePOZ = true;
end
prm.kPen = 1000;          % penalty ($/h per MW of power-balance error or zone violation)
prm.caseNo = caseNo;
end
