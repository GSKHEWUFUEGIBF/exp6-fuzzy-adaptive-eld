function R = run_fpso(prm, o)
%RUN_FPSO  Particle swarm optimisation whose inertia weight w is set every iteration
%   by the fuzzy supervisor o.fis (inputs: swarm diversity and stagnation).
rng(o.seed, 'twister');
range = prm.hi - prm.lo;   n = numel(range);
x = eld_repair(prm.lo + rand(o.N, n).*range, prm);
v = zeros(o.N, n);   vmax = 0.2*range;
f = eld_evaluate(x, prm);
pb = x;   pbf = f;   [gbf, ig] = min(f);   gb = x(ig,:);
c1 = 1.5;  c2 = 1.5;
opt = evalfisOptions('OutOfRangeInputValueMessage','none','NoRuleFiredMessage','none', ...
                     'EmptyOutputFuzzySetMessage','none');
H = zeros(o.G, 7);  stall = 0;  tic;
for g = 1:o.G
    D = pop_diversity(x, prm);   S = min(stall/20, 1);
    w = evalfis(o.fis, [D S], opt);
    v = w*v + c1*rand(o.N,n).*(pb - x) + c2*rand(o.N,n).*(gb - x);
    v = max(min(v, vmax), -vmax);
    x = eld_repair(x + v, prm);
    f = eld_evaluate(x, prm);
    imp = f < pbf;   pb(imp,:) = x(imp,:);   pbf(imp) = f(imp);
    [m, ig] = min(pbf);
    if m < gbf - 1e-3, stall = 0; else, stall = stall + 1; end
    if m < gbf, gbf = m; gb = pb(ig,:); end
    H(g,:) = [gbf mean(f) w NaN NaN D S];
end
R.time = toc;
R.fit = gbf;   R.P = gb;
[~, R.F, R.L, R.mis, R.viol] = eld_evaluate(R.P, prm);
R.hist = H;
end
