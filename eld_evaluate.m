function [fit, F, L, mis, viol] = eld_evaluate(P, prm)
%#codegen
%ELD_EVALUATE  Fitness of a population of dispatches (one dispatch per row).
%   F    : total fuel cost ($/h)
%   L    : transmission loss from Kron's B-coefficient formula (MW)
%   mis  : power-balance error  sum(P) - PD - PL (MW)
%   viol : depth of operation inside prohibited zones (MW)
%   fit  : F + kPen*(|mis| + viol)      -> minimised by the GA
N = size(P,1);
F = sum(prm.a + prm.b.*P + prm.c.*P.^2, 2);
L = sum((P*prm.B).*P, 2) + P*prm.B0.' + prm.B00;
mis = sum(P,2) - prm.PD - L;
viol = zeros(N,1);
if prm.usePOZ
    for z = 1:2
        zl = prm.POZ(:,2*z-1).';   zh = prm.POZ(:,2*z).';
        inside = (P > zl) & (P < zh);
        depth  = min(P - zl, zh - P);
        viol = viol + sum(inside.*depth, 2);
    end
end
fit = F + prm.kPen*(abs(mis) + viol);
end
