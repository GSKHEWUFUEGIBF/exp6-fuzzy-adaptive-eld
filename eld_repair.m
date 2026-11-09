function P = eld_repair(P, prm)
%#codegen
%ELD_REPAIR  Make every dispatch (row of P) satisfy  sum(P) = PD + PL  within the unit limits.
%   The mismatch is shared equally among the units that still have room to move;
%   the loss is recomputed and the step repeated until the error is below 1e-6 MW.
lo = prm.lo;  hi = prm.hi;
P = min(max(P, lo), hi);
for it = 1:60
    L = sum((P*prm.B).*P, 2) + P*prm.B0.' + prm.B00;
    d = prm.PD + L - sum(P,2);                 % MW still to be generated (+) or removed (-)
    if max(abs(d)) < 1e-6, break; end
    free = ((d > 0) & (P < hi - 1e-9)) | ((d < 0) & (P > lo + 1e-9));
    nf = sum(free, 2);   nf(nf == 0) = 1;
    P = P + free.*(d./nf);
    P = min(max(P, lo), hi);
end
end
