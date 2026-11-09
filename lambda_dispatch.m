function out = lambda_dispatch(prm)
%LAMBDA_DISPATCH  Classical equal-incremental-cost (lambda-iteration) dispatch with losses.
%   Coordination equation of unit i:   b_i + 2 c_i P_i = lambda (1 - dPL/dP_i)
%   with dPL/dP_i = 2 sum_j B_ij P_j + B0_i. For a given lambda the outputs are found
%   by Gauss-Seidel iteration (limits enforced), and lambda is adjusted by bisection
%   until  sum(P) = PD + PL.
n = numel(prm.a);
P = (prm.lo + prm.hi)/2;
lamLo = 1;  lamHi = 40;
hist = zeros(0,3);
for k = 1:200
    lam = (lamLo + lamHi)/2;
    for inner = 1:500
        Pold = P;
        for i = 1:n
            s = 2*(prm.B(i,:)*P.' - prm.B(i,i)*P(i));
            P(i) = (lam*(1 - prm.B0(i) - s) - prm.b(i)) / (2*(prm.c(i) + lam*prm.B(i,i)));
            P(i) = min(max(P(i), prm.lo(i)), prm.hi(i));
        end
        if max(abs(P - Pold)) < 1e-10, break; end
    end
    L = P*prm.B*P.' + prm.B0*P.' + prm.B00;
    d = prm.PD + L - sum(P);
    hist(end+1,:) = [lam d L]; %#ok<AGROW>
    if abs(d) < 1e-8, break; end
    if d > 0, lamLo = lam; else, lamHi = lam; end
end
out.P = P;  out.lambda = lam;  out.L = L;  out.mis = sum(P) - prm.PD - L;
out.F = sum(prm.a + prm.b.*P + prm.c.*P.^2);
out.iter = k;  out.hist = hist;
end
