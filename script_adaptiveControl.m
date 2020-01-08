% A very fast timescale of human motor adaptation: within movement
% adjustments of internal representations during reaching
%
% Crevecoeur F., Thonnard J.-L., lefèvre P.

% This script reproduces the simualtions shown in Fig. 6.

%%
% Experiment 1
%--------------------------------------------------------------------------

% Simulated 10 trials per condition of online learning rate
nt = 10
forces1 = zeros(nt,60);
forces25 = zeros(nt,60);
forces50 = zeros(nt,60);

allx1 = zeros(nt,60);
allx25 = zeros(nt,60);
allx50 = zeros(nt,60);

alldx1 = zeros(nt,60);
alldx25 = zeros(nt,60);
alldx50 = zeros(nt,60);

alldy1 = zeros(nt,60);
alldy25 = zeros(nt,60);
alldy50 = zeros(nt,60);

ux = zeros(nt,60,2);
uy = zeros(nt,60,2);

for i = 1:nt
    
    simout = adaptiveReaching([13 0],[0.1 0],0,25,[]);
    forces1(i,:) = simout.state(5,:);
    allx1(i,:) = simout.state(1,:);
    alldy1(i,:) = simout.state(4,:);
    
    simout = adaptiveReaching([13 0],[0.2 0],0,25,[]);
    forces25(i,:) = simout.state(5,:);
    allx25(i,:) = simout.state(1,:);
    alldy25(i,:) = simout.state(4,:);
    
    simout = adaptiveReaching([13 0],[0.5 0],0,25,[]);
    forces50(i,:) = simout.state(5,:);
    allx50(i,:) = simout.state(1,:);
    alldy50(i,:) = simout.state(4,:);
    
end

% close all,
figure
plot(mean(forces1))
hold on, plot(mean(forces1)-std(forces1))
hold on, plot(mean(forces1)+std(forces1))
plot(mean(forces25),'k')
hold on, plot(mean(forces25)-std(forces25),'k')
hold on, plot(mean(forces25)+std(forces25),'k')
plot(mean(forces50),'k')
hold on, plot(mean(forces50)-std(forces50),'k')
hold on, plot(mean(forces50)+std(forces50),'k')
xlabel('Time (iteration number)');
ylabel('Control force [N]');
