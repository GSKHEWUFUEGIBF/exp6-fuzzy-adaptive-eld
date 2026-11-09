function R = run_ga(prm, o)
%RUN_GA  One complete run of the standard GA (o.mode = 'fixed') or of the
%   hybrid fuzzy-adaptive GA (o.mode = 'fuzzy').
%   o.N population size, o.G generations, o.pc / o.pm fixed rates (standard GA),
%   o.fis fuzzy supervisor (fuzzy GA), o.seed random seed.
%   R.hist columns: best fitness, mean fitness, pc, pm, mutation step, diversity D, stagnation S
rng(o.seed, 'twister');
range = prm.hi - prm.lo;
pop = eld_repair(prm.lo + rand(o.N, numel(range)).*range, prm);
fit = eld_evaluate(pop, prm);
opt = evalfisOptions('OutOfRangeInputValueMessage','none','NoRuleFiredMessage','none', ...
                     'EmptyOutputFuzzySetMessage','none');
H = zeros(o.G, 7);
if isfield(o,'tol'), tol = o.tol; else, tol = 1e-3; end
best = min(fit);  stall = 0;  tic;
for g = 1:o.G
    D = pop_diversity(pop, prm);
    S = min(stall/20, 1);
    if strcmp(o.mode, 'fuzzy')
        y = evalfis(o.fis, [D S], opt);   pc = y(1);   pm = y(2);   sig = y(3);
    else
        pc = o.pc;   pm = o.pm;
        sig = 0.1*(1 - (g-1)/o.G) + 0.002;           % standard GA: step shrinks linearly with time
    end
    [pop, fit] = ga_generation(pop, fit, pc, pm, sig, prm);
    if min(fit) < best - tol, stall = 0; else, stall = stall + 1; end
    best = min(best, min(fit));
    H(g,:) = [min(fit) mean(fit) pc pm sig D S];
end
R.time = toc;
[R.fit, ib] = min(fit);   R.P = pop(ib,:);
[~, R.F, R.L, R.mis, R.viol] = eld_evaluate(R.P, prm);
R.hist = H;
end
