% A very fast timescale of human motor adaptation: within movement
% adjustments of internal representations during reaching
%
% Crevecoeur F., Thonnard J.-L., lefèvre P.

% This script reproduces the simualtions shown in Fig. 6. Different version
% or packages in Matlab my produce errors. Users should not hesitate to
% contact in case problems arise on their setup:
% frederic.crevecoeur@uclouvain.be
%
% The code was tested on Mac OS 10.12.6, Matlab R2017a

%%
% Experiment 1
%--------------------------------------------------------------------------

nt = 10; % Number of Simulated Trials
forces1 = zeros(nt,60);
forces25 = zeros(nt,60);
forces50 = zeros(nt,60);

forces_costUp = zeros(nt,60);
forces_costDown = zeros(nt,60);

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
    
    simout = adaptiveReaching([13 0 0],[0.1 0],0,25,[]);
    forces1(i,:) = simout.state(5,:);
    allx1(i,:) = simout.state(1,:);
    alldy1(i,:) = simout.state(4,:);
    
    simout = adaptiveReaching([13 0 0],[0.2 0],0,25,[]);
    forces25(i,:) = simout.state(5,:);
    allx25(i,:) = simout.state(1,:);
    alldy25(i,:) = simout.state(4,:);
    
    simout = adaptiveReaching([13 0 0],[0.5 0],0,25,[]);
    forces50(i,:) = simout.state(5,:);
    allx50(i,:) = simout.state(1,:);
    alldy50(i,:) = simout.state(4,:);
    

    %Change in Cost
    simout = adaptiveReaching([13 0 1.03],[0 0],0,25,[]);
    forces_costUp(i,:) = simout.state(5,:);
    simout = adaptiveReaching([13 0 0.98],[0 0],0,25,[]);
    forces_costDown(i,:) = simout.state(5,:);
    
    
end

% close all,
figure
subplot(121)
plot(mean(forces1),'k'), hold on
plot(mean(forces25),'r')
plot(mean(forces50),'b')
legend('\gamma = 0.1','\gamma = 0.2','\gamma = 0.5')
xlabel('Time (iteration number)');
ylabel('Control force [N]');

subplot(122)
plot(mean(forces_costUp),'k'), hold on
plot(mean(forces_costDown),'Color',[.7 .7 .7])
legend('Cost up','Cost down');
xlabel('Time (iteration number)');
ylabel('Control force [N]');
