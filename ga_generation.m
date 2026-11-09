function [pop, fit] = ga_generation(pop, fit, pc, pm, sig, prm)
%#codegen
%GA_GENERATION  One generation of the real-coded genetic algorithm.
%   pop : N x 6 population (one dispatch per row),  fit : N x 1 fitness
%   pc  : crossover probability,  pm : mutation probability (per gene)
%   sig : mutation step as a fraction of each unit's range
%   Operators: elitism (2) -> binary tournament selection -> BLX-0.3 crossover
%              -> Gaussian mutation -> limit clipping and power-balance repair.
[N, n] = size(pop);
nE = 2;
range = prm.hi - prm.lo;

% elitism: the two best dispatches pass unchanged
[~, idx] = sort(fit);
elite = pop(idx(1:nE), :);   eliteFit = fit(idx(1:nE));

% selection: binary tournament
i1 = randi(N, N, 1);   i2 = randi(N, N, 1);
win = i1;   better = fit(i2) < fit(i1);   win(better) = i2(better);
kids = pop(win, :);

% crossover: blend crossover BLX-alpha, parent genes are mixed into two children
alpha = 0.3;
for k = 1:2:N-1
    if rand < pc
        p1 = kids(k,:);   p2 = kids(k+1,:);
        dmin = min(p1,p2);   I = max(p1,p2) - dmin;
        kids(k,:)   = dmin - alpha*I + rand(1,n).*((1+2*alpha)*I);
        kids(k+1,:) = dmin - alpha*I + rand(1,n).*((1+2*alpha)*I);
    end
end

% mutation: Gaussian step on each gene with probability pm
M = rand(N, n) < pm;
kids = kids + M.*randn(N, n).*(sig*range);

% constraint handling and evaluation
kids = eld_repair(kids, prm);
kfit = eld_evaluate(kids, prm);

% new generation = elites + best N-2 children
[~, kidx] = sort(kfit);
pop = [elite; kids(kidx(1:N-nE), :)];
fit = [eliteFit; kfit(kidx(1:N-nE))];
end
