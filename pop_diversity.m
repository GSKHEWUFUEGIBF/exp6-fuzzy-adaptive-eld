function D = pop_diversity(pop, prm)
%#codegen
%POP_DIVERSITY  Normalised population diversity, 0 (all identical) ... 1 (uniformly spread).
%   Mean standard deviation of each unit's output divided by its range; a uniform
%   random population has std/range = 1/sqrt(12) = 0.2887, which is mapped to 1.
s = std(pop, 0, 1)./(prm.hi - prm.lo);
D = min(mean(s)/0.2887, 1);
end
