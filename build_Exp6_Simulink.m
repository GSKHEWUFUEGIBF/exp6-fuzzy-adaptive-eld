%% build_Exp6_Simulink.m
% Experiment 6 : Economic load dispatch using a hybrid fuzzy adaptive genetic algorithm
% Builds the Simulink model Exp6_FuzzyGA_ELD_Sim.slx, in which ONE SIMULATION STEP IS
% ONE GA GENERATION:
%
%   GA_Engine (MATLAB Function: selection, crossover, mutation, repair, evaluation)
%        |  diversity D, stagnation S
%        v
%   Fuzzy_Supervisor (Fuzzy Logic Controller block, 9 Mamdani rules) --> [pc pm sigma]
%        |                                   Fixed_Parameters (standard GA) --+
%        v                                                                     |
%   Mode_Switch (fuzzy / fixed) --> Unit_Delay --> back to GA_Engine <---------+
%
%   GA_Engine best dispatch --> Dispatch_Check (fuel cost, Kron loss, power balance,
%   prohibited-zone check, built from Simulink blocks) --> displays, scopes, logging
%
% The model runs Case 3 (800 MW, ramp limits and prohibited zones) and is simulated with
% the fuzzy supervisor and with fixed rates for the same 30 seeds as the MATLAB study.
% Requires MATLAB, Simulink and the Fuzzy Logic Toolbox.   >> build_Exp6_Simulink

clc; close all;
here   = fileparts(mfilename('fullpath'));   if isempty(here), here = pwd; end
addpath(here);
mdl    = 'Exp6_FuzzyGA_ELD_Sim';
figDir = fullfile(here,'figures_simulink');  if ~exist(figDir,'dir'), mkdir(figDir); end
logFile = fullfile(here,'Exp6_simulink_run.log');  if exist(logFile,'file'), delete(logFile); end
diary(logFile);  diary on;

NPOP = 30;  NGEN = 200;  NRUN = 30;
sys = eld6_data(800);   prm = eld_problem(sys, 2);
ref = poz_exact(prm);
fisGA = build_fis();

%% 1. Create the model and fill its workspace
if bdIsLoaded(mdl), close_system(mdl,0); end
slxFile = fullfile(here,[mdl '.slx']);
if exist(slxFile,'file'), delete(slxFile); end
new_system(mdl);   open_system(mdl);
set_param(mdl,'Solver','FixedStepDiscrete','FixedStep','1','StartTime','0','StopTime',num2str(NGEN-1));
ws = get_param(mdl,'ModelWorkspace');
assignin(ws,'Exp6_prm',prm);          assignin(ws,'Exp6_fis',fisGA);
assignin(ws,'Exp6_N',NPOP);           assignin(ws,'Exp6_G',NGEN);
assignin(ws,'Exp6_seed',1);           assignin(ws,'Exp6_useFuzzy',1);
assignin(ws,'Exp6_IC',[0.8; 0.05; 0.102]);
assignin(ws,'Exp6_PD',prm.PD);        assignin(ws,'Exp6_a',prm.a(:));  assignin(ws,'Exp6_b',prm.b(:));
assignin(ws,'Exp6_c',prm.c(:));       assignin(ws,'Exp6_B',prm.B);     assignin(ws,'Exp6_B0',prm.B0);
assignin(ws,'Exp6_B00',prm.B00);

load_system('fuzblock');
fuzzyBlk = ['fuzblock/Fuzzy Logic ' newline 'Controller'];

%% 2. GA engine (MATLAB Function block)
gaBlk = [mdl '/GA_Engine'];
add_block('simulink/User-Defined Functions/MATLAB Function', gaBlk, 'Position', [200 140 380 340]);
code = strjoin({
'function [bestCost, bestP, D, S] = GA_Engine(ctrl, Exp6_prm, Exp6_N, Exp6_seed)'
'% One call = one generation of the real-coded GA.'
'% ctrl = [pc; pm; sigma] chosen by the fuzzy supervisor (or the fixed values)'
'% for this generation. Outputs describe the population after the generation.'
'prm = Exp6_prm;  N = Exp6_N;'
'persistent pop fit best stall'
'if isempty(pop)'
'    rng(Exp6_seed, ''twister'');'
'    range = prm.hi - prm.lo;'
'    pop = eld_repair(prm.lo + rand(N, numel(range)).*range, prm);'
'    fit = eld_evaluate(pop, prm);'
'    best = min(fit);   stall = 0;'
'end'
'[pop, fit] = ga_generation(pop, fit, ctrl(1), ctrl(2), ctrl(3), prm);'
'[bestCost, ib] = min(fit);'
'if bestCost < best - 1e-3, stall = 0; else, stall = stall + 1; end'
'best = min(best, bestCost);'
'bestP = pop(ib, :).'';'
'D = pop_diversity(pop, prm);'
'S = min(stall/20, 1);'
'end'}, newline);
rt = sfroot;   ch = rt.find('-isa','Stateflow.EMChart','Path',gaBlk);
ch.Script = code;
pars = {'Exp6_prm'; 'Exp6_N'; 'Exp6_seed'};
for k = 1:3
    d = ch.find('-isa','Stateflow.Data','Name',pars{k});
    d.Scope = 'Parameter';   d.Tunable = (k == 3);
end
dS = ch.find('-isa','Stateflow.Data','Name','ctrl');  dS.Props.Array.Size = '3';
try, set_param(gaBlk,'ContentPreviewEnabled','off'); catch, end

%% 3. Fuzzy supervisor, fixed parameters, mode switch, delay
addb(mdl,'simulink/Signal Routing/Mux','Mux_DS',[470 470 475 540],'Inputs','2');
addb(mdl,fuzzyBlk,'Fuzzy_Supervisor',[540 475 670 535],'FIS','Exp6_fis', ...
     'OutOfRangeInputValueMessage','none','NoRuleFiredMessage','none','EmptyOutputFuzzySetMessage','none');

sub = [mdl '/Fixed_Parameters'];
addb(mdl,'simulink/Ports & Subsystems/Subsystem','Fixed_Parameters',[540 600 670 660]);
Simulink.SubSystem.deleteContents(sub);
add_block('simulink/Sources/Constant',[sub '/pc_fixed'],'Value','0.8','Position',[60 30 120 60]);
add_block('simulink/Sources/Constant',[sub '/pm_fixed'],'Value','0.05','Position',[60 90 120 120]);
add_block('simulink/Sources/Digital Clock',[sub '/Generation_Clock'],'SampleTime','1','Position',[40 150 100 180]);
add_block('simulink/User-Defined Functions/Fcn',[sub '/Step_Schedule'], ...
          'Expression','0.1*(1-(u(1)+1)/200)+0.002','Position',[150 150 330 180]);
add_block('simulink/Signal Routing/Mux',[sub '/Mux_fixed'],'Inputs','3','Position',[390 20 395 190]);
add_block('simulink/Sinks/Out1',[sub '/pc_pm_sigma'],'Position',[450 98 480 112]);
add_line(sub,'pc_fixed/1','Mux_fixed/1','autorouting','smart');
add_line(sub,'pm_fixed/1','Mux_fixed/2','autorouting','smart');
add_line(sub,'Generation_Clock/1','Step_Schedule/1','autorouting','smart');
add_line(sub,'Step_Schedule/1','Mux_fixed/3','autorouting','smart');
add_line(sub,'Mux_fixed/1','pc_pm_sigma/1','autorouting','smart');

addb(mdl,'simulink/Sources/Constant','Use_Fuzzy',[560 700 650 730],'Value','Exp6_useFuzzy');
addb(mdl,'simulink/Signal Routing/Switch','Mode_Switch',[760 480 810 660],'Criteria','u2 > Threshold','Threshold','0.5');
addb(mdl,'simulink/Discrete/Unit Delay','Unit_Delay',[90 225 140 265],'Orientation','right','InitialCondition','Exp6_IC','SampleTime','1');

%% 4. Dispatch check subsystem (built from Simulink blocks)
dc = [mdl '/Dispatch_Check'];
addb(mdl,'simulink/Ports & Subsystems/Subsystem','Dispatch_Check',[560 150 710 300]);
Simulink.SubSystem.deleteContents(dc);
add_block('simulink/Sources/In1',[dc '/P'],'Position',[20 200 50 214]);
% fuel cost  F = sum(a + b.*P + c.*P.^2)
add_block('simulink/Math Operations/Math Function',[dc '/P_squared'],'Operator','square','Position',[130 30 180 60]);
add_block('simulink/Math Operations/Gain',[dc '/c_coeff'],'Gain','Exp6_c','Multiplication','Element-wise(K.*u)','Position',[230 30 290 60]);
add_block('simulink/Math Operations/Gain',[dc '/b_coeff'],'Gain','Exp6_b','Multiplication','Element-wise(K.*u)','Position',[230 90 290 120]);
add_block('simulink/Sources/Constant',[dc '/a_coeff'],'Value','Exp6_a','Position',[230 140 290 170]);
add_block('simulink/Math Operations/Sum',[dc '/Unit_costs'],'Inputs','+++','IconShape','rectangular','Position',[340 70 370 140]);
add_block('simulink/Math Operations/Sum of Elements',[dc '/Total_cost_sum'],'Position',[410 90 450 120]);
add_block('simulink/Sinks/Out1',[dc '/Fuel_cost'],'Position',[520 98 550 112]);
% loss  PL = P'*B*P + B0*P + B00
add_block('simulink/Math Operations/Gain',[dc '/B_matrix'],'Gain','Exp6_B','Multiplication','Matrix(K*u)','Position',[130 230 190 260]);
add_block('simulink/Math Operations/Dot Product',[dc '/P_B_P'],'Position',[240 220 280 270]);
add_block('simulink/Math Operations/Gain',[dc '/B0_row'],'Gain','Exp6_B0','Multiplication','Matrix(K*u)','Position',[230 300 290 330]);
add_block('simulink/Sources/Constant',[dc '/B00_const'],'Value','Exp6_B00','Position',[230 360 290 390]);
add_block('simulink/Math Operations/Sum',[dc '/Kron_loss'],'Inputs','+++','IconShape','rectangular','Position',[340 260 370 380]);
add_block('simulink/Sinks/Out1',[dc '/Loss'],'Position',[520 313 550 327]);
% power balance  e = sum(P) - PD - PL
add_block('simulink/Math Operations/Sum of Elements',[dc '/Total_generation'],'Position',[130 440 170 470]);
add_block('simulink/Sources/Constant',[dc '/Load_demand'],'Value','Exp6_PD','Position',[230 480 290 510]);
add_block('simulink/Math Operations/Sum',[dc '/Balance'],'Inputs','+--','IconShape','rectangular','Position',[340 430 370 520]);
add_block('simulink/Sinks/Out1',[dc '/Balance_error'],'Position',[520 468 550 482]);
add_block('simulink/Sinks/Out1',[dc '/Generation'],'Position',[520 548 550 562]);
% prohibited-zone check
add_block('simulink/User-Defined Functions/MATLAB Function',[dc '/Zone_Check'],'Position',[230 570 330 620]);
zc = rt.find('-isa','Stateflow.EMChart','Path',[dc '/Zone_Check']);
zc.Script = strjoin({'function viol = Zone_Check(P, Exp6_prm)', ...
    '% depth (MW) of operation inside prohibited zones, 0 = all units in allowed segments', ...
    '[~, ~, ~, ~, viol] = eld_evaluate(P.'', Exp6_prm);', 'end'}, newline);
dz = zc.find('-isa','Stateflow.Data','Name','Exp6_prm');  dz.Scope = 'Parameter';  dz.Tunable = false;
add_block('simulink/Sinks/Out1',[dc '/Zone_violation'],'Position',[520 588 550 602]);
lns = {'P/1','P_squared/1'; 'P_squared/1','c_coeff/1'; 'P/1','b_coeff/1'; 'c_coeff/1','Unit_costs/1'; ...
       'b_coeff/1','Unit_costs/2'; 'a_coeff/1','Unit_costs/3'; 'Unit_costs/1','Total_cost_sum/1'; ...
       'Total_cost_sum/1','Fuel_cost/1'; 'P/1','B_matrix/1'; 'P/1','P_B_P/1'; 'B_matrix/1','P_B_P/2'; ...
       'P/1','B0_row/1'; 'P_B_P/1','Kron_loss/1'; 'B0_row/1','Kron_loss/2'; 'B00_const/1','Kron_loss/3'; ...
       'Kron_loss/1','Loss/1'; 'P/1','Total_generation/1'; 'Total_generation/1','Balance/1'; ...
       'Load_demand/1','Balance/2'; 'Kron_loss/1','Balance/3'; 'Balance/1','Balance_error/1'; ...
       'Total_generation/1','Generation/1'; 'P/1','Zone_Check/1'; 'Zone_Check/1','Zone_violation/1'};
for k = 1:size(lns,1), add_line(dc, lns{k,1}, lns{k,2}, 'autorouting', 'smart'); end

%% 5. Displays, scopes and logging
addb(mdl,'simulink/Sinks/Display','Fuel_cost_per_h',[800 120 920 150]);
addb(mdl,'simulink/Sinks/Display','Loss_MW',[800 170 920 200]);
addb(mdl,'simulink/Sinks/Display','Balance_error_MW',[800 220 920 250]);
addb(mdl,'simulink/Sinks/Display','Generation_MW',[800 270 920 300]);
addb(mdl,'simulink/Sinks/Display','Zone_violation_MW',[800 320 920 350]);
addb(mdl,'simulink/Sinks/Display','Dispatch_P1_to_P6_MW',[560 330 640 430]);
addb(mdl,'simulink/Sinks/Scope','Scope_Best_Cost',[560 30 600 70]);
addb(mdl,'simulink/Sinks/Scope','Scope_Dispatch',[460 330 500 370]);
addb(mdl,'simulink/Sinks/Scope','Scope_Fuzzy_Inputs',[460 560 500 600]);
addb(mdl,'simulink/Sinks/Scope','Scope_GA_Parameters',[900 480 940 520]);
addb(mdl,'simulink/Sinks/To Workspace','Log_Cost',[650 35 730 65],'VariableName','log_cost','SaveFormat','Array');
addb(mdl,'simulink/Sinks/To Workspace','Log_P',[440 400 520 430],'VariableName','log_P','SaveFormat','Array');
addb(mdl,'simulink/Sinks/To Workspace','Log_DS',[440 620 520 650],'VariableName','log_DS','SaveFormat','Array');
addb(mdl,'simulink/Sinks/To Workspace','Log_Ctrl',[880 560 960 590],'VariableName','log_ctrl','SaveFormat','Array');
addb(mdl,'simulink/Signal Routing/Mux','Mux_Check',[990 120 995 350],'Inputs','5');
addb(mdl,'simulink/Sinks/To Workspace','Log_Check',[1030 220 1110 250],'VariableName','log_check','SaveFormat','Array');

%% 6. Connections
ln(mdl,'Unit_Delay/1','GA_Engine/1');
ln(mdl,'GA_Engine/1','Scope_Best_Cost/1');     ln(mdl,'GA_Engine/1','Log_Cost/1');
ln(mdl,'GA_Engine/2','Dispatch_Check/1');      ln(mdl,'GA_Engine/2','Scope_Dispatch/1');
ln(mdl,'GA_Engine/2','Dispatch_P1_to_P6_MW/1'); ln(mdl,'GA_Engine/2','Log_P/1');
ln(mdl,'GA_Engine/3','Mux_DS/1');              ln(mdl,'GA_Engine/4','Mux_DS/2');
ln(mdl,'Mux_DS/1','Fuzzy_Supervisor/1');       ln(mdl,'Mux_DS/1','Scope_Fuzzy_Inputs/1');   ln(mdl,'Mux_DS/1','Log_DS/1');
ln(mdl,'Fuzzy_Supervisor/1','Mode_Switch/1');  ln(mdl,'Use_Fuzzy/1','Mode_Switch/2');       ln(mdl,'Fixed_Parameters/1','Mode_Switch/3');
ln(mdl,'Mode_Switch/1','Unit_Delay/1');        ln(mdl,'Mode_Switch/1','Scope_GA_Parameters/1'); ln(mdl,'Mode_Switch/1','Log_Ctrl/1');
ln(mdl,'Dispatch_Check/1','Fuel_cost_per_h/1'); ln(mdl,'Dispatch_Check/2','Loss_MW/1');
ln(mdl,'Dispatch_Check/3','Balance_error_MW/1'); ln(mdl,'Dispatch_Check/4','Generation_MW/1');
ln(mdl,'Dispatch_Check/5','Zone_violation_MW/1');
for k = 1:5, ln(mdl,sprintf('Dispatch_Check/%d',k),sprintf('Mux_Check/%d',k)); end
ln(mdl,'Mux_Check/1','Log_Check/1');

note(mdl,'Experiment 6 - ELD by hybrid fuzzy adaptive GA   (1 simulation step = 1 GA generation, Case 3: 800 MW)',[90 -10]);
note(mdl,'Fuzzy supervision loop:  D, S  ->  pc, pm, sigma  ->  next generation',[200 395]);
note(mdl,'Use_Fuzzy = 1: hybrid fuzzy GA,   0: standard GA',[520 745]);
set_param(mdl,'ZoomFactor','FitSystem');
save_system(mdl, slxFile);
fprintf('Created %s\n', slxFile);

%% 7. Model images
try
    print(['-s' mdl],'-dpng','-r300',fullfile(figDir,'model_top_level.png'));
    open_system(dc);  set_param(dc,'ZoomFactor','FitSystem');
    print(['-s' dc],'-dpng','-r300',fullfile(figDir,'model_dispatch_check.png'));  close_system(dc);
    open_system(sub); set_param(sub,'ZoomFactor','FitSystem');
    print(['-s' sub],'-dpng','-r300',fullfile(figDir,'model_fixed_parameters.png'));  close_system(sub);
catch ME
    warning('Model image export failed: %s', ME.message);
end

%% 8. Simulate: fuzzy supervisor and fixed rates, 30 seeds each, compare with MATLAB
opt = evalfisOptions('OutOfRangeInputValueMessage','none','NoRuleFiredMessage','none','EmptyOutputFuzzySetMessage','none');
fin = zeros(NRUN,2);  maxDiff = zeros(NRUN,2);  chk = zeros(NRUN,5,2);  T = cell(NRUN,2);
for mode = 1:2                       % 1 = fuzzy, 0 = fixed
    useF = double(mode == 1);
    for r = 1:NRUN
        rng(r,'twister');  range = prm.hi - prm.lo;
        pop0 = eld_repair(prm.lo + rand(NPOP,6).*range, prm);
        if useF, IC = evalfis(fisGA, [pop_diversity(pop0,prm) 0], opt).'; else, IC = [0.8; 0.05; 0.102]; end
        in = Simulink.SimulationInput(mdl);
        in = in.setVariable('Exp6_seed', r, 'Workspace', mdl);
        in = in.setVariable('Exp6_useFuzzy', useF, 'Workspace', mdl);
        in = in.setVariable('Exp6_IC', IC, 'Workspace', mdl);
        so = sim(in);
        c = so.log_cost(:);  P = squeeze(so.log_P);  if size(P,1) == 6, P = P.'; end
        fin(r,mode) = c(end);   chk(r,:,mode) = so.log_check(end,:);
        T{r,mode} = struct('cost',c,'P',P,'DS',so.log_DS,'ctrl',so.log_ctrl);
        o = struct('N',NPOP,'G',NGEN,'mode','fixed','pc',0.8,'pm',0.05,'fis',fisGA,'seed',r);
        if useF, o.mode = 'fuzzy'; end
        R = run_ga(prm, o);
        maxDiff(r,mode) = max(abs(R.hist(:,1) - c));
    end
end
nm = {'Fuzzy-adaptive GA','Standard GA'};
fprintf('\nSimulink, Case 3 (exact optimum %.4f $/h), %d seeds each\n', ref.F, NRUN);
S = struct();
for mode = 1:2
    e = fin(:,mode) - ref.F;
    fprintf('  %-18s best %.4f  mean %.4f  worst %.4f  within 1 $/h %2d/%d\n', nm{mode}, min(fin(:,mode)), mean(fin(:,mode)), max(fin(:,mode)), sum(e < 1), NRUN);
    fprintf('  %-18s max |Simulink - MATLAB| best-cost trajectory difference over all runs: %.3e $/h\n', '', max(maxDiff(:,mode)));
    S.mode(mode) = struct('name',nm{mode},'best',min(fin(:,mode)),'mean',mean(fin(:,mode)),'worst',max(fin(:,mode)), ...
        'within1',sum(e < 1),'maxDiffVsMatlab',max(maxDiff(:,mode)), ...
        'maxAbsBalance',max(abs(chk(:,3,mode))),'maxZoneViol',max(chk(:,5,mode)));
end
[~, ib] = min(fin(:,1));   last = chk(ib,:,1);
fprintf('  Dispatch_Check of the best fuzzy-GA run (seed %d): cost %.4f $/h, loss %.4f MW, generation %.4f MW, balance error %.2e MW, zone violation %.2f MW\n', ...
        ib, last(1), last(2), last(4), last(3), last(5));
S.bestSeed = ib;  S.bestCheck = last;  S.bestP = T{ib,1}.P(end,:);  S.refF = ref.F;
fid = fopen(fullfile(here,'simulink_results.json'),'w');  fprintf(fid,'%s',jsonencode(S,'PrettyPrint',true));  fclose(fid);

%% 9. Result figure from the logged signals (example seed used in the report)
M = load(fullfile(here,'Exp6_results.mat'),'out');  rx = M.out.exampleRun;
cBlue = [42 120 214]/255;  cOrange = [235 104 52]/255;  cAqua = [27 175 122]/255;  cGrey = [82 81 78]/255;  cViolet = [124 92 196]/255;
col = [cBlue; cOrange; cAqua; cViolet; cGrey; [196 160 30]/255];
f = figure('Position',[60 60 1000 720],'Color','w','DefaultAxesFontName','Times New Roman', ...
           'DefaultTextFontName','Times New Roman','DefaultAxesFontSize',11);
try, theme(f,'light'); catch, end
tiledlayout(f,2,2,'TileSpacing','compact','Padding','compact');
g = (1:NGEN)';  F = T{rx,1};  X = T{rx,2};
nexttile;  semilogy(g, max(X.cost - ref.F,1e-3), 'Color', cGrey, 'LineWidth', 1.8);  hold on;
semilogy(g, max(F.cost - ref.F,1e-3), 'Color', cBlue, 'LineWidth', 1.8);
grid on; box off; xlim([1 NGEN]); xlabel('Generation (simulation step)'); ylabel('Best cost - optimum ($/h)');
legend({'Use\_Fuzzy = 0 (standard GA)','Use\_Fuzzy = 1 (fuzzy-adaptive GA)'},'Box','off','Location','southwest');
title(sprintf('(a) Scope\\_Best\\_Cost, seed %d', rx), 'FontWeight', 'normal');
nexttile;  hold on;
for i = 1:6, plot(g, F.P(:,i), 'Color', col(i,:), 'LineWidth', 1.6); end
grid on; box off; xlim([1 NGEN]); xlabel('Generation (simulation step)'); ylabel('Output (MW)');
legend(compose('P_%d',1:6),'Box','off','Location','eastoutside');
title('(b) Scope\_Dispatch (fuzzy-adaptive GA)', 'FontWeight', 'normal');
nexttile;  DS = squeeze(F.DS);  if size(DS,1) == 2, DS = DS.'; end
plot(g, DS(:,1), 'Color', cAqua, 'LineWidth', 1.6);  hold on;  plot(g, DS(:,2), 'Color', cOrange, 'LineWidth', 1.6);
grid on; box off; xlim([1 NGEN]); ylim([0 1.05]); xlabel('Generation (simulation step)'); ylabel('Fuzzy inputs');
legend({'Diversity D','Stagnation S'},'Box','off','Location','southoutside','Orientation','horizontal');
title('(c) Scope\_Fuzzy\_Inputs', 'FontWeight', 'normal');
nexttile;  U = squeeze(F.ctrl);  if size(U,1) == 3, U = U.'; end
plot(g, U(:,1), 'Color', cBlue, 'LineWidth', 1.6);  hold on;  plot(g, U(:,2), 'Color', cOrange, 'LineWidth', 1.6);
plot(g, U(:,3), 'Color', cViolet, 'LineWidth', 1.6);
grid on; box off; xlim([1 NGEN]); ylim([0 1.05]); xlabel('Generation (simulation step)'); ylabel('Supervisor outputs');
legend({'p_c','p_m','\sigma_m'},'Box','off','Location','southoutside','Orientation','horizontal');
title('(d) Scope\_GA\_Parameters (fuzzy-adaptive GA)', 'FontWeight', 'normal');
drawnow;  print(f, fullfile(figDir,'sim_results.png'), '-dpng', '-r300');
save_system(mdl);
fprintf('Figures saved in %s\n', figDir);
diary off;

%% ------------------------------------------------------------------ local functions
function addb(mdl,lib,name,pos,varargin)
add_block(lib,[mdl '/' name],'Position',pos,varargin{:});
end

function ln(mdl,src,dst)
add_line(mdl,src,dst,'autorouting','smart');
end

function note(mdl,txt,pos)
a = Simulink.Annotation(mdl,txt);
a.Position = pos;   a.FontName = 'Times New Roman';   a.FontSize = 12;
end
