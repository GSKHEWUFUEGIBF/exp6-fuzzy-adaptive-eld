function out = poz_exact(prm)
%POZ_EXACT  Exact global optimum of the dispatch with prohibited operating zones.
%   Each unit's allowed range splits into 3 disjoint segments, so the feasible set is the
%   union of 3^6 = 729 boxes. On every box the problem is convex and is solved exactly by
%   lambda-iteration; the cheapest feasible box gives the global optimum.
n = numel(prm.a);
seg = cell(1,n);
for i = 1:n
    z = prm.POZ(i,:);
    s = [prm.lo(i) z(1); z(2) z(3); z(4) prm.hi(i)];
    s(:,1) = max(s(:,1), prm.lo(i));  s(:,2) = min(s(:,2), prm.hi(i));
    seg{i} = s(s(:,2) >= s(:,1), :);              % keep only non-empty segments
end
ns = cellfun(@(s) size(s,1), seg);
best.F = inf;  nFeas = 0;  nBox = prod(ns);
q = prm;  q.usePOZ = false;
for k = 1:nBox
    sub = cell(1,n);  [sub{:}] = ind2sub(ns, k);
    lo = zeros(1,n);  hi = zeros(1,n);
    for i = 1:n, lo(i) = seg{i}(sub{i},1);  hi(i) = seg{i}(sub{i},2); end
    q.lo = lo;  q.hi = hi;
    % quick feasibility test: balance must be reachable inside the box
    Llo = lo*q.B*lo.' + q.B0*lo.' + q.B00;   Lhi = hi*q.B*hi.' + q.B0*hi.' + q.B00;
    if sum(lo) - Llo > prm.PD + 1 || sum(hi) - Lhi < prm.PD - 1, continue; end
    r = lambda_dispatch(q);
    if abs(r.mis) < 1e-6
        nFeas = nFeas + 1;
        if r.F < best.F, best = r; end
    end
end
out = best;  out.nBox = nBox;  out.nFeas = nFeas;
end
