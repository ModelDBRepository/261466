% A very fast timescale of human motor adaptation: within movement
% adjustments of internal representations during reaching
%
% Crevecoeur F., Thonnard J.-L., lefèvre P.

% This script reproduces the simualtions shown in Fig. 6

%%
% Experiment 2
%--------------------------------------------------------------------------

simout = adaptiveReaching([0 0 0],[.05 0],.8,25,[]);
close all;
h1 = figure;
h2 = figure;

for i = 1:5
    
    figure(h1)
    simout = adaptiveReaching([0 0 0],[0.5 0],.6,25,simout);
    simout = adaptiveReaching([0 0 0],[0.5 0],.6,25,simout);
    
    figure(h2)
    simout = adaptiveReaching([0 0 0],[0.5 0],.6,25,simout);
    simout = adaptiveReaching([13 0 0],[0.5 0],.6,25,simout);
    
    figure(h1)
    simout = adaptiveReaching([0 0 0],[0.5 0],.6,25,simout);
    simout = adaptiveReaching([0 0 0],[0.5 0],.6,25,simout);
    
    figure(h2)
    simout = adaptiveReaching([0 0 0],[0.5 0],.6,25,simout);
    simout = adaptiveReaching([-13 0 0],[0.5 0],.6,25,simout);
    
end

close(h1)

