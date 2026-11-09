function [fisGA, fisPSO] = build_fis()
%BUILD_FIS  Mamdani fuzzy supervisors that tune the search parameters on line.
%   Inputs (both):  D = normalised population diversity   (0 ... 1)
%                   S = normalised stagnation, generations without improvement / 20  (0 ... 1)
%   fisGA  outputs: pc (crossover probability, 0.5 ... 1.0), pm (mutation probability, 0 ... 0.3)
%                   and ms (mutation step as a fraction of the unit range, 0 ... 0.2)
%   fisPSO output : w  (inertia weight, 0.4 ... 0.9)
%   min/max inference, centroid defuzzification.

fisGA = mamfis('Name','FuzzyGA_supervisor','AndMethod','min','OrMethod','max', ...
               'ImplicationMethod','min','AggregationMethod','max','DefuzzificationMethod','centroid');
fisGA = addInputs(fisGA);
fisGA = addOutput(fisGA,[0.5 1.0],'Name','pc');
fisGA = addMF(fisGA,'pc','trimf',[0.5 0.5 0.7],'Name','Low');
fisGA = addMF(fisGA,'pc','trimf',[0.6 0.75 0.9],'Name','Medium');
fisGA = addMF(fisGA,'pc','trimf',[0.8 1.0 1.0],'Name','High');
fisGA = addOutput(fisGA,[0 0.3],'Name','pm');
fisGA = addMF(fisGA,'pm','trimf',[0 0 0.1],'Name','Low');
fisGA = addMF(fisGA,'pm','trimf',[0.05 0.15 0.25],'Name','Medium');
fisGA = addMF(fisGA,'pm','trimf',[0.2 0.3 0.3],'Name','High');
fisGA = addOutput(fisGA,[0 0.2],'Name','ms');           % mutation step, fraction of unit range
fisGA = addMF(fisGA,'ms','trimf',[0 0 0.02],'Name','Small');
fisGA = addMF(fisGA,'ms','trimf',[0.01 0.05 0.1],'Name','Medium');
fisGA = addMF(fisGA,'ms','trimf',[0.06 0.2 0.2],'Name','Large');
% rule table  [D S pc pm ms]   1 = Low/Small, 2 = Medium, 3 = High/Large
R = [1 1 2 2 1;   1 2 1 3 2;   1 3 1 3 3;
     2 1 3 1 1;   2 2 2 2 2;   2 3 2 3 3;
     3 1 3 1 2;   3 2 3 1 2;   3 3 2 2 2];
fisGA = addRule(fisGA,[R ones(9,1) ones(9,1)]);

fisPSO = mamfis('Name','FuzzyPSO_supervisor','AndMethod','min','OrMethod','max', ...
                'ImplicationMethod','min','AggregationMethod','max','DefuzzificationMethod','centroid');
fisPSO = addInputs(fisPSO);
fisPSO = addOutput(fisPSO,[0.4 0.9],'Name','w');
fisPSO = addMF(fisPSO,'w','trimf',[0.4 0.4 0.6],'Name','Low');
fisPSO = addMF(fisPSO,'w','trimf',[0.5 0.65 0.8],'Name','Medium');
fisPSO = addMF(fisPSO,'w','trimf',[0.7 0.9 0.9],'Name','High');
Rw = [1 1 2;  1 2 3;  1 3 3;
      2 1 1;  2 2 2;  2 3 3;
      3 1 1;  3 2 2;  3 3 3];
fisPSO = addRule(fisPSO,[Rw ones(9,1) ones(9,1)]);
end

function fis = addInputs(fis)
fis = addInput(fis,[0 1],'Name','Diversity');
fis = addMF(fis,'Diversity','trapmf',[0 0 0.05 0.2],'Name','Low');
fis = addMF(fis,'Diversity','trimf',[0.05 0.2 0.4],'Name','Medium');
fis = addMF(fis,'Diversity','trapmf',[0.2 0.4 1 1],'Name','High');
fis = addInput(fis,[0 1],'Name','Stagnation');
fis = addMF(fis,'Stagnation','trapmf',[0 0 0.1 0.4],'Name','Low');
fis = addMF(fis,'Stagnation','trimf',[0.1 0.4 0.7],'Name','Medium');
fis = addMF(fis,'Stagnation','trapmf',[0.4 0.7 1 1],'Name','High');
end
