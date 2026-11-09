%% Exp6_FuzzyGA_ELD.m
% Experiment 6 : Economic load dispatch using a hybrid fuzzy adaptive genetic algorithm
% Objective    : minimise the total fuel cost of the IEEE 6-unit thermal system while
%                generation = load demand + transmission loss.
%
%  1. Test system and fuel-cost characteristics
%  2. Classical solution: lambda-iteration (equal incremental cost), checked with fmincon
%  3. Fuzzy supervisors (Mamdani) for the GA and for PSO
%  4. Statistical comparison, 30 independent runs: standard GA, fuzzy-adaptive GA, fuzzy PSO
%       Case 1  PD = 1263 MW, limits + losses                         (convex)
%       Case 2  PD = 1263 MW, + ramp-rate limits + prohibited zones   (non-convex benchmark)
%       Case 3  PD =  800 MW, + ramp-rate limits + prohibited zones   (zones active)
%  5. Sensitivity of the standard GA to its fixed pc, pm
%  6. Load sweep
%  7. Validation against published results
%  8. Figures and result files
%
% Requires MATLAB with the Optimization and Fuzzy Logic Toolboxes.   >> Exp6_FuzzyGA_ELD

clc; close all;
here   = fileparts(mfilename('fullpath'));   if isempty(here), here = pwd; end
figDir = fullfile(here,'figures_matlab');    if ~exist(figDir,'dir'), mkdir(figDir); end
logFile = fullfile(here,'Exp6_run.log');     if exist(logFile,'file'), delete(logFile); end
diary(logFile);  diary on;
fprintf('Experiment 6 - ELD by hybrid fuzzy adaptive GA      %s\n', char(datetime('now')));

cBlue = [42 120 214]/255;  cOrange = [235 104 52]/255;  cAqua = [27 175 122]/255;
cGrey = [82 81 78]/255;    cLight = [200 198 192]/255;  cViolet = [124 92 196]/255;
set(groot,'DefaultAxesFontName','Times New Roman','DefaultTextFontName','Times New Roman', ...
    'DefaultAxesFontSize',11,'DefaultLegendFontName','Times New Roman');

NRUN = 30;  NPOP = 30;  NGEN = 200;            % 30 runs, 30 individuals, 200 generations
GAfix = [0.8 0.05];                             % standard GA: pc, pm

%% 1. Test system
sys = eld6_data(1263);
fprintf('\n1. %s, PD = %g MW\n', sys.name, sys.PD);
fprintf('   unit   a($/h)  b($/MWh)  c($/MW2h)  Pmin  Pmax   P0   UR   DR   prohibited zones (MW)\n');
for i = 1:6
    fprintf('   %d   %6.0f  %7.1f  %9.4f  %5d %5d %5d %4d %4d   [%d %d] [%d %d]\n', i, sys.a(i), sys.b(i), ...
        sys.c(i), sys.Pmin(i), sys.Pmax(i), sys.P0(i), sys.UR(i), sys.DR(i), sys.POZ(i,:));
end

f = figure('Position',[60 60 980 400],'Color','w');  try, theme(f,'light'); catch, end
tl = tiledlayout(f,1,2,'TileSpacing','compact','Padding','compact');
col = [cBlue; cOrange; cAqua; cViolet; cGrey; [196 160 30]/255];
nexttile;  hold on;
for i = 1:6
    P = linspace(sys.Pmin(i), sys.Pmax(i), 100);
    plot(P, sys.a(i) + sys.b(i)*P + sys.c(i)*P.^2, 'Color', col(i,:), 'LineWidth', 1.8);
end
grid on; box off; xlabel('Unit output P_i (MW)'); ylabel('Fuel cost F_i ($/h)');
title('(a) Fuel-cost characteristics'); legend(compose('Unit %d',1:6),'Location','northwest','Box','off');
nexttile;  hold on;
for i = 1:6
    P = linspace(sys.Pmin(i), sys.Pmax(i), 100);
    plot(P, sys.b(i) + 2*sys.c(i)*P, 'Color', col(i,:), 'LineWidth', 1.8);
end
grid on; box off; xlabel('Unit output P_i (MW)'); ylabel('Incremental cost dF_i/dP_i ($/MWh)');
title('(b) Incremental-cost characteristics');
savefig_(f, fullfile(figDir,'fig_cost_curves.png'));

%% 2. Classical lambda-iteration (Case 1) and fmincon check
p1 = eld_problem(sys,1);
lam = lambda_dispatch(p1);
obj = @(P) sum(p1.a + p1.b.*P + p1.c.*P.^2);
con = @(P) deal([], sum(P) - p1.PD - (P*p1.B*P.' + p1.B0*P.' + p1.B00));
xo  = fmincon(obj,(p1.lo+p1.hi)/2,[],[],[],[],p1.lo,p1.hi,con, optimoptions('fmincon','Display','off', ...
             'Algorithm','sqp','OptimalityTolerance',1e-12,'ConstraintTolerance',1e-10,'StepTolerance',1e-14));
[~, Fmc, Lmc, mmc] = eld_evaluate(xo, p1);
fprintf('\n2. Lambda-iteration: lambda = %.5f $/MWh, %d lambda updates\n', lam.lambda, lam.iter);
fprintf('   P = [%s] MW\n', sprintf('%.4f ', lam.P));
fprintf('   cost %.4f $/h, loss %.4f MW, balance error %.2e MW\n', lam.F, lam.L, lam.mis);
fprintf('   fmincon (SQP): cost %.4f $/h, loss %.4f MW, max |dP| vs lambda = %.2e MW\n', Fmc, Lmc, max(abs(xo-lam.P)));

%% 3. Fuzzy supervisors
[fisGA, fisPSO] = build_fis();
writeFIS(fisGA, fullfile(here,'fuzzyGA_supervisor'));
writeFIS(fisPSO, fullfile(here,'fuzzyPSO_supervisor'));
fprintf('\n3. Fuzzy supervisors: %d rules (GA), %d rules (PSO)\n', numel(fisGA.Rules), numel(fisPSO.Rules));
plot_fis(fisGA, fisPSO, figDir, cBlue, cOrange, cAqua, cGrey);

%% 4. Statistical comparison
caseDef = {1, 1263, 'Case 1: 1263 MW, limits + losses';
           2, 1263, 'Case 2: 1263 MW, + ramp limits + prohibited zones';
           2,  800, 'Case 3: 800 MW, + ramp limits + prohibited zones'};
meth = {'Standard GA','Fuzzy-adaptive GA','Fuzzy PSO'};
res = struct();
for c = 1:3
    s = eld6_data(caseDef{c,2});   prm = eld_problem(s, caseDef{c,1});
    if caseDef{c,1} == 1, ref = lambda_dispatch(prm); refName = 'lambda-iteration';
    else,                 ref = poz_exact(prm);       refName = sprintf('exhaustive search of %d convex sub-regions', ref.nBox); end
    fprintf('\n4.%d %s\n    reference optimum (%s): %.4f $/h, loss %.4f MW\n', c, caseDef{c,3}, refName, ref.F, ref.L);
    fprintf('    P_ref = [%s]\n', sprintf('%.4f ', ref.P));
    C.prm = prm;  C.ref = ref;  C.title = caseDef{c,3};
    for m = 1:3
        fin = zeros(NRUN,1);  P = zeros(NRUN,6);  L = zeros(NRUN,1);  mis = L;  viol = L;  tm = L;  gReach = nan(NRUN,1);
        conv = zeros(NGEN, NRUN);
        for r = 1:NRUN
            o = struct('N',NPOP,'G',NGEN,'mode','fixed','pc',GAfix(1),'pm',GAfix(2),'fis',fisGA,'seed',r);
            switch m
                case 1, R = run_ga(prm, o);
                case 2, o.mode = 'fuzzy';  R = run_ga(prm, o);
                case 3, o.fis = fisPSO;    R = run_fpso(prm, o);
            end
            fin(r) = R.fit;  P(r,:) = R.P;  L(r) = R.L;  mis(r) = R.mis;  viol(r) = R.viol;  tm(r) = R.time;
            conv(:,r) = R.hist(:,1);
            k = find(R.hist(:,1) - ref.F < 1, 1);  if ~isempty(k), gReach(r) = k; end
            if r == 1, C.example{m} = R; end
            if m == 2, C.allHist{r} = R.hist; end
        end
        [~, ib] = min(fin);
        st = struct('best',min(fin),'mean',mean(fin),'worst',max(fin),'std',std(fin), ...
                    'within1',sum(fin - ref.F < 1),'genTo1',mean(gReach,'omitnan'),'time',mean(tm), ...
                    'Pbest',P(ib,:),'Lbest',L(ib),'misBest',mis(ib),'violBest',viol(ib), ...
                    'maxAbsMis',max(abs(mis)),'maxViol',max(viol));
        C.stat{m} = st;  C.final{m} = fin;  C.conv{m} = conv;
        fprintf('    %-18s best %.4f  mean %.4f  worst %.4f  std %.4f  | runs within 1 $/h: %2d/%d  mean gen to 1 $/h: %5.1f  | t = %.3f s\n', ...
            meth{m}, st.best, st.mean, st.worst, st.std, st.within1, NRUN, st.genTo1, st.time);
        fprintf('    %-18s best P = [%s]  loss %.4f  |mis|max %.1e  zone viol max %.2e\n', '', ...
            sprintf('%.4f ', st.Pbest), st.Lbest, st.maxAbsMis, st.maxViol);
    end
    res.case(c) = C;
end

%% 5. Sensitivity of the standard GA to fixed pc and pm (Case 3)
pcSet = [0.6 0.8 0.95];  pmSet = [0.01 0.05 0.2];
prm3 = res.case(3).prm;  F3 = res.case(3).ref.F;
sens = zeros(numel(pcSet), numel(pmSet), 3);   % mean excess, worst excess, runs within 1 $/h
fprintf('\n5. Standard GA with fixed parameters, Case 3 (%d runs each): mean / worst excess cost ($/h)\n', NRUN);
for i = 1:numel(pcSet)
    for j = 1:numel(pmSet)
        fin = zeros(NRUN,1);
        for r = 1:NRUN
            o = struct('N',NPOP,'G',NGEN,'mode','fixed','pc',pcSet(i),'pm',pmSet(j),'fis',fisGA,'seed',r);
            R = run_ga(prm3, o);   fin(r) = R.fit;
        end
        sens(i,j,:) = [mean(fin)-F3, max(fin)-F3, sum(fin-F3 < 1)];
        fprintf('    pc = %.2f  pm = %.2f :  mean %.3f  worst %.3f  within 1 $/h %2d/%d\n', pcSet(i), pmSet(j), sens(i,j,1), sens(i,j,2), sens(i,j,3), NRUN);
    end
end
sF = res.case(3).stat{2};
fprintf('    fuzzy-adaptive GA :  mean %.3f  worst %.3f  within 1 $/h %2d/%d\n', sF.mean-F3, sF.worst-F3, sF.within1, NRUN);

%% 6. Load sweep (ramp limits + prohibited zones)
PDs = 800:50:1250;   NS = 10;
sweep = zeros(numel(PDs), 7);   % PD, lambda cost, lambda zone violation, exact, FGA best, FGA mean, FGA worst
fprintf('\n6. Load sweep with ramp limits and prohibited zones (%d fuzzy-GA runs per load)\n', NS);
for k = 1:numel(PDs)
    s = eld6_data(PDs(k));   prm = eld_problem(s,2);
    q = prm;  q.usePOZ = false;   lm = lambda_dispatch(q);
    [~,~,~,~,v] = eld_evaluate(lm.P, prm);
    ex = poz_exact(prm);
    fin = zeros(NS,1);
    for r = 1:NS
        o = struct('N',NPOP,'G',NGEN,'mode','fuzzy','fis',fisGA,'seed',100+r);
        R = run_ga(prm, o);  fin(r) = R.fit;
    end
    sweep(k,:) = [PDs(k) lm.F v ex.F min(fin) mean(fin) max(fin)];
    fprintf('    PD %4d MW: lambda %.3f (inside zones by %.2f MW) | exact %.3f | FGA best %.3f mean %.3f worst %.3f\n', sweep(k,:));
end

%% 7. Published results for Case 2, re-evaluated with the same data
lit = {'PSO (Gaing 2003)',   [447.4970 173.3221 263.4745 139.0594 165.4761 87.1280], 15449.882, 12.9584;
       'NPSO-LRS',           [446.9600 173.3944 262.3436 139.5120 164.7089 89.0162], 15450.000, 12.9351;
       'IPSO-TVAC',          [447.5840 173.2010 263.3310 138.8520 165.3280 87.1500], 15443.063, 12.4460;
       'BFO',                [449.4600 172.8800 263.4100 143.4900 164.9100 81.2520], 15443.850, 12.4020;
       'SSA',                [448.39165 169.30115 256.19797 139.74938 170.27317 89.72839], 15419.803, 10.6421};
prm2 = res.case(2).prm;
fprintf('\n7. Published dispatches for Case 2 re-evaluated (cost $/h, loss MW, balance error MW)\n');
litTab = zeros(size(lit,1), 6);
for k = 1:size(lit,1)
    [~, F, L, mis, v] = eld_evaluate(lit{k,2}, prm2);
    litTab(k,:) = [lit{k,3} lit{k,4} F L mis v];
    fprintf('    %-18s reported %.3f / %.4f | recomputed %.3f / %.4f | balance error %+.4f | zone viol %.2f\n', lit{k,1}, litTab(k,:));
end

%% 8. Figures
plot_results(res, sens, pcSet, pmSet, sweep, figDir, NGEN, cBlue, cOrange, cAqua, cGrey, cLight, cViolet);

% result files for the report and for the Simulink model
out.lambda = struct('P',lam.P,'F',lam.F,'L',lam.L,'lambda',lam.lambda,'iter',lam.iter,'mis',lam.mis);
out.fmincon = struct('P',xo,'F',Fmc,'L',Lmc,'mis',mmc);
for c = 1:3
    C = res.case(c);
    out.case(c).title = C.title;  out.case(c).refF = C.ref.F;  out.case(c).refP = C.ref.P;  out.case(c).refL = C.ref.L;
    for m = 1:3, out.case(c).stat(m) = C.stat{m}; end
end
out.exampleRun = ir_example(res);
out.sens = sens;  out.sweep = sweep;  out.lit = litTab;  out.litNames = lit(:,1)';
out.settings = struct('NRUN',NRUN,'NPOP',NPOP,'NGEN',NGEN,'pc',GAfix(1),'pm',GAfix(2));
fid = fopen(fullfile(here,'matlab_results.json'),'w');  fprintf(fid,'%s',jsonencode(out,'PrettyPrint',true));  fclose(fid);
save(fullfile(here,'Exp6_results.mat'),'res','out','fisGA','fisPSO','sys');
fprintf('\nResults saved: matlab_results.json, Exp6_results.mat; figures in %s\n', figDir);
diary off;

%% ------------------------------------------------------------------ local functions
function savefig_(f, file)
ax = findall(f, 'Type', 'axes');
for k = 1:numel(ax), try, ax(k).Toolbar.Visible = 'off'; catch, end, end
drawnow;
print(f, file, '-dpng', '-r300');
end

function plot_fis(fisGA, fisPSO, figDir, cBlue, cOrange, cAqua, cGrey)
cols = [cBlue; cOrange; cAqua];
f = figure('Position',[60 60 1000 640],'Color','w');  try, theme(f,'light'); catch, end
tiledlayout(f,2,3,'TileSpacing','compact','Padding','compact');
items = {'input',1,'(a) Input 1: population diversity D'; 'input',2,'(b) Input 2: stagnation S';
         'output',1,'(c) Output: crossover probability p_c'; 'output',2,'(d) Output: mutation probability p_m';
         'output',3,'(e) Output: mutation step \sigma_m (fraction of range)'};
for k = 1:5
    nexttile;  hold on;
    if strcmp(items{k,1},'input'), v = fisGA.Inputs(items{k,2}); else, v = fisGA.Outputs(items{k,2}); end
    x = linspace(v.Range(1), v.Range(2), 400);
    for j = 1:numel(v.MembershipFunctions)
        y = evalmf(v.MembershipFunctions(j), x);
        plot(x, y, 'Color', cols(j,:), 'LineWidth', 1.8);
    end
    grid on; box off; ylim([0 1.1]); xlim(v.Range); ylabel('Membership');
    title(items{k,3}, 'FontWeight', 'normal');
    legend({v.MembershipFunctions.Name}, 'Location', 'southoutside', 'Orientation', 'horizontal', 'Box', 'off', 'FontSize', 9);
end
nexttile;  axis off;
labs = {'L','M','H'};  rt = zeros(3,3,3);
R = fisGA.Rules;
for k = 1:numel(R)
    a = R(k).Antecedent;  cq = R(k).Consequent;  rt(a(1), a(2), :) = cq;
end
rowName = {'Low','Medium','High'};
txt = {'Consequents: pc / pm / step', '', 'D \ S      Low       Medium    High'};
for i = 1:3
    txt{end+1} = sprintf('%-8s  %s/%s/%s     %s/%s/%s     %s/%s/%s', rowName{i}, ...
        labs{rt(i,1,1)}, labs{rt(i,1,2)}, labs{rt(i,1,3)}, labs{rt(i,2,1)}, labs{rt(i,2,2)}, labs{rt(i,2,3)}, ...
        labs{rt(i,3,1)}, labs{rt(i,3,2)}, labs{rt(i,3,3)}); %#ok<AGROW>
end
txt{end+1} = '';  txt{end+1} = 'L = Low/Small, M = Medium,';  txt{end+1} = 'H = High/Large (9 rules)';
text(0, 0.95, txt, 'VerticalAlignment', 'top', 'FontName', 'Courier New', 'FontSize', 10, 'Color', 'k', 'Interpreter', 'none');
title('(f) Rule base', 'FontWeight', 'normal');
savefig_(f, fullfile(figDir,'fig_fis_mfs.png'));

% control surfaces
f = figure('Position',[60 60 1050 330],'Color','w');  try, theme(f,'light'); catch, end
tiledlayout(f,1,4,'TileSpacing','compact','Padding','compact');
[D, S] = meshgrid(linspace(0,1,41));
opt = evalfisOptions('OutOfRangeInputValueMessage','none','NoRuleFiredMessage','none','EmptyOutputFuzzySetMessage','none');
Y = evalfis(fisGA, [D(:) S(:)], opt);   W = evalfis(fisPSO, [D(:) S(:)], opt);
Z = {Y(:,1), Y(:,2), Y(:,3), W};
zl = {'p_c','p_m','\sigma_m','w (PSO)'};  tt = {'(a) Crossover probability','(b) Mutation probability','(c) Mutation step','(d) PSO inertia weight'};
for k = 1:4
    nexttile;  surf(D, S, reshape(Z{k}, size(D)), 'EdgeAlpha', 0.25);  colormap(parula);
    xlabel('Diversity D'); ylabel('Stagnation S'); zlabel(zl{k}); title(tt{k}, 'FontWeight', 'normal');
    view(-130, 28);  axis tight;  grid on;
end
savefig_(f, fullfile(figDir,'fig_fis_surfaces.png'));
end

function plot_results(res, sens, pcSet, pmSet, sweep, figDir, NGEN, cBlue, cOrange, cAqua, cGrey, cLight, cViolet)
mc = [cGrey; cBlue; cOrange];   g = 1:NGEN;
meth = {'Standard GA (p_c = 0.8, p_m = 0.05)','Fuzzy-adaptive GA','Fuzzy PSO'};
% convergence: median excess cost over 30 runs, with interquartile band
f = figure('Position',[60 60 1080 360],'Color','w');  try, theme(f,'light'); catch, end
tiledlayout(f,1,3,'TileSpacing','compact','Padding','compact');
for c = 1:3
    nexttile;  hold on;  C = res.case(c);  h = gobjects(1,3);
    for m = 1:3
        E = max(C.conv{m} - C.ref.F, 1e-3);
        q = prctile(E, [25 50 75], 2);
        fill([g fliplr(g)], [q(:,1)' fliplr(q(:,3)')], mc(m,:), 'FaceAlpha', 0.12, 'EdgeColor', 'none');
        h(m) = plot(g, q(:,2), 'Color', mc(m,:), 'LineWidth', 1.8);
    end
    set(gca, 'YScale', 'log');  grid on; box off;  xlim([1 NGEN]);
    xlabel('Generation');  if c == 1, ylabel('Best cost - optimum ($/h)'); end
    title(sprintf('(%s) %s', char('a'+c-1), C.title), 'FontWeight', 'normal', 'FontSize', 10);
    if c == 1, legend(h, meth, 'Location', 'northeast', 'Box', 'off', 'FontSize', 9); end
end
savefig_(f, fullfile(figDir,'fig_convergence.png'));

% parameter adaptation in one fuzzy-GA run (Case 3)
Fopt = res.case(3).ref.F;  AH = res.case(3).allHist;
score = cellfun(@(h) (h(40,1) - Fopt) * (h(end,1) - Fopt < 0.2), AH);   % trapped at gen 40, solved at the end
[~, ir] = max(score);  H = AH{ir};
f = figure('Position',[60 60 900 640],'Color','w');  try, theme(f,'light'); catch, end
tiledlayout(f,3,1,'TileSpacing','compact','Padding','compact');
nexttile;  semilogy(g, max(H(:,1)-Fopt,1e-3), 'Color', cBlue, 'LineWidth', 1.8);
grid on; box off; xlim([1 NGEN]); ylabel('Best cost - optimum ($/h)');
title(sprintf('(a) Convergence of fuzzy-adaptive GA run %d, Case 3', ir), 'FontWeight', 'normal');
nexttile;  plot(g, H(:,6), 'Color', cAqua, 'LineWidth', 1.8);  hold on;  plot(g, H(:,7), 'Color', cOrange, 'LineWidth', 1.8);
grid on; box off; xlim([1 NGEN]); ylim([0 1.05]); ylabel('Fuzzy inputs');
legend({'Diversity D','Stagnation S'}, 'Box', 'off', 'Location', 'east');
title('(b) Inputs of the fuzzy supervisor', 'FontWeight', 'normal');
nexttile;  plot(g, H(:,3), 'Color', cBlue, 'LineWidth', 1.8);  hold on;
plot(g, H(:,4), 'Color', cOrange, 'LineWidth', 1.8);  plot(g, H(:,5), 'Color', cViolet, 'LineWidth', 1.8);
yline(0.8, '--', 'Color', cBlue, 'LineWidth', 1);  yline(0.05, '--', 'Color', cOrange, 'LineWidth', 1);
grid on; box off; xlim([1 NGEN]); ylim([0 1.05]); xlabel('Generation'); ylabel('GA parameters');
legend({'p_c','p_m','\sigma_m','p_c of standard GA','p_m of standard GA'}, 'Box', 'off', 'Location', 'east', 'NumColumns', 2);
title('(c) Parameters set by the fuzzy supervisor', 'FontWeight', 'normal');
savefig_(f, fullfile(figDir,'fig_adaptation.png'));

% distribution of final costs over 30 runs (Cases 2 and 3)
f = figure('Position',[60 60 900 360],'Color','w');  try, theme(f,'light'); catch, end
tiledlayout(f,1,2,'TileSpacing','compact','Padding','compact');
for c = 2:3
    nexttile;  hold on;  C = res.case(c);
    for m = 1:3
        e = C.final{m} - C.ref.F;
        x = m + 0.18*(rand(numel(e),1) - 0.5);
        scatter(x, e, 22, mc(m,:), 'filled', 'MarkerFaceAlpha', 0.7);
        plot([m-0.25 m+0.25], [median(e) median(e)], 'Color', cGrey, 'LineWidth', 2);
    end
    set(gca, 'XTick', 1:3, 'XTickLabel', {'Standard GA','Fuzzy GA','Fuzzy PSO'});  xlim([0.5 3.5]);
    grid on; box off;  ylabel('Final cost - optimum ($/h)');
    title(sprintf('(%s) %s', char('a'+c-2), C.title), 'FontWeight', 'normal', 'FontSize', 10);
end
savefig_(f, fullfile(figDir,'fig_final_costs.png'));

% best dispatch of Case 3 with limits and prohibited zones
C = res.case(3);  prm = C.prm;
f = figure('Position',[60 60 900 380],'Color','w');  try, theme(f,'light'); catch, end
ax = axes(f);  hold(ax,'on');
for i = 1:6
    rectangle(ax,'Position',[i-0.4 prm.lo(i) 0.8 prm.hi(i)-prm.lo(i)],'FaceColor',[0.95 0.95 0.94],'EdgeColor',cLight);
    for z = 1:2
        zl = max(prm.POZ(i,2*z-1), prm.lo(i));  zh = min(prm.POZ(i,2*z), prm.hi(i));
        if zh > zl, rectangle(ax,'Position',[i-0.4 zl 0.8 zh-zl],'FaceColor',[1 0.85 0.8],'EdgeColor','none'); end
    end
end
lamU = lambda_dispatch(setfield(prm,'usePOZ',false)); %#ok<SFLD>
h1 = plot(ax, (1:6)-0.2, lamU.P, 'o', 'MarkerSize', 7, 'MarkerFaceColor', cGrey, 'MarkerEdgeColor', 'w');
h2 = plot(ax, (1:6), C.ref.P, 's', 'MarkerSize', 8, 'MarkerFaceColor', cAqua, 'MarkerEdgeColor', 'w');
h3 = plot(ax, (1:6)+0.2, C.stat{2}.Pbest, 'd', 'MarkerSize', 7, 'MarkerFaceColor', cBlue, 'MarkerEdgeColor', 'w');
h4 = patch(ax, NaN, NaN, [1 0.85 0.8], 'EdgeColor', 'none');  h5 = patch(ax, NaN, NaN, [0.95 0.95 0.94], 'EdgeColor', cLight);
set(ax,'XTick',1:6,'XTickLabel',compose('Unit %d',1:6));  xlim(ax,[0.4 6.6]);  grid(ax,'on');  box(ax,'off');
ylabel(ax,'Output (MW)');
legend(ax,[h1 h2 h3 h5 h4],{'\lambda-iteration ignoring zones (infeasible)','Exact optimum','Fuzzy-adaptive GA (best run)', ...
       'Allowed range (limits and ramp)','Prohibited zone'},'Location','northeast','Box','off','FontSize',9);
title(ax,'Dispatch for Case 3 (800 MW)','FontWeight','normal');
savefig_(f, fullfile(figDir,'fig_dispatch_case3.png'));

% sensitivity map and load sweep
f = figure('Position',[60 60 1000 420],'Color','w');  try, theme(f,'light'); catch, end
tiledlayout(f,1,2,'TileSpacing','compact','Padding','compact');
nexttile;  imagesc(sens(:,:,1));  colormap(gca, flipud(gray(256))*0.6 + 0.4);  cb = colorbar;  cb.Label.String = 'Mean excess cost ($/h)';
for i = 1:3, for j = 1:3
    text(j, i, sprintf('%.2f\n(%d/30)', sens(i,j,1), sens(i,j,3)), 'HorizontalAlignment','center', 'FontSize', 10, 'Color', 'k');
end, end
set(gca,'XTick',1:3,'XTickLabel',compose('%.2f',pmSet),'YTick',1:3,'YTickLabel',compose('%.2f',pcSet));
xlabel('Fixed mutation probability p_m');  ylabel('Fixed crossover probability p_c');
title('(a) Standard GA, Case 3: mean excess cost (runs within 1 $/h)', 'FontWeight', 'normal', 'FontSize', 10);
nexttile;  hold on;
yyaxis left;  b = bar(sweep(:,1), sweep(:,3), 0.5, 'FaceColor', [1 0.75 0.68], 'EdgeColor', 'none');
ylabel('\lambda-iteration: depth inside zones (MW)');  set(gca,'YColor',cGrey);
yyaxis right;  plot(sweep(:,1), sweep(:,6) - sweep(:,4), '-o', 'Color', cBlue, 'LineWidth', 1.6, 'MarkerFaceColor', cBlue);
plot(sweep(:,1), sweep(:,7) - sweep(:,4), '--s', 'Color', cOrange, 'LineWidth', 1.2);
ylabel('Fuzzy GA cost - exact optimum ($/h)');  set(gca,'YColor',cGrey);
xlabel('Load demand P_D (MW)');  grid on; box off;
legend({'\lambda-iteration zone violation','Fuzzy GA mean (10 runs)','Fuzzy GA worst (10 runs)'},'Location','southoutside','Orientation','horizontal','Box','off','FontSize',9);
title('(b) Load sweep with ramp limits and prohibited zones', 'FontWeight', 'normal', 'FontSize', 10);
savefig_(f, fullfile(figDir,'fig_sensitivity_sweep.png'));
end

function ir = ir_example(res)
Fopt = res.case(3).ref.F;  AH = res.case(3).allHist;
score = cellfun(@(h) (h(40,1) - Fopt) * (h(end,1) - Fopt < 0.2), AH);
[~, ir] = max(score);
end
