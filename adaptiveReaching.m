function simout = adaptiveReaching(loads,gamma,tvia,buildup,simin)

%SIMOUT = ADAPTIVEREACHING(LOADS,GAMMA,PERTURBATION,TVIA,BUILDUP,SIMIN)
%
%   LOADS: 1x2 vector with load parameter. The force field is
%   LOAD[1]*forward velocity, the expected load parameter is LOAD[2].
%   LOAD[3] is the parameter used to increase or decrease the cost for the
%   simulations of Fig. 6c. 
%   GAMMA: Online learning rate for A and B matrices. Only A is update in
%   this paper. 
%   TVIA: Time at which the point mass must be at the via-point. When TVIA
%   is zero, the simulation time is set to 0.6s (default). When TVIA is
%   non-zero, simulation time is set to 1sec, and the coordiante of the
%   point-mass is constrained at TVIA.
%   BUILDUP: Exponent of polynomial function for cost buildup (25).
%   SIMIN: Input simulation data contains, either empty or herited from
%   previous simulation run
%
%   SIMOUT: Output simulation data, contains:
%       .state:     state vector over time
%       .FBgains:   time series of optimal control gains
%       .estimate:  time series of estimated state vectors
%       .control:   time series of control vectors
%       .AestCont:  time series of estimated model matrix
%       .BestCont:  time series of estimated model matrix, both in
%                   continuous time representation



% script_adaptiveControl
Gx = .1;
Gy = Gx;
m = 2.5;
tau = 0.1;
L = loads(1); % load variable
lambda = 0;
k = 0;


A = [0 0 1 0 0 0;0 0 0 1 0 0;-k/m 0 -Gx/m L/m m^-1 0;...
    0 -k/m 0 -Gy/m 0 m^-1;0 0 0 0 -tau^-1 lambda/tau;0 0 0 0 lambda/tau -tau^-1];

ANull = [0 0 1 0 0 0;0 0 0 1 0 0;-k/m 0 -Gx/m 0/m m^-1 0;...
    0 -k/m 0 -Gy/m 0 m^-1;0 0 0 0 -tau^-1 lambda/tau;0 0 0 0 lambda/tau -tau^-1]; % For via-point

B = [0 0;0 0;0 0;0 0;tau^-1 lambda/tau; lambda/tau tau^-1];
%A

simdata.A = A;
simdata.ANull = ANull;
% simdata.AClamp = AClamp;

simdata.B = B;
xp = mvnrnd(zeros(size(A)),eye(6));
AParam = -eye(6);

% load AStable
Aest = A;

if isempty(simin)
    Aest(3,4) = loads(2)/m;
else
    Aest(3:4,1:4) = simin.AestCont(3:4,1:4,end);
end

simdata.AestCont = Aest;
simdata.BestCont = B;
simdata.AParamCont = AParam;

% Simulation time corresponds to Exp 1 or 2
if tvia == 0
    simdata.time = .6;
    xfinal = [0 .15 0 0 0 0]'; % Experiment 1
else
    simdata.time = 1;
    xfinal = [0 .16 0 0 0 0]'; % VP

end

simdata.tvia = tvia;
simdata.stab = .01;
simdata.delta= .01;
simdata.m = m;

% Cost and Learning Rate
simdata.alpha = [1000 1000 20 20 0 0];
if ~isempty(buildup)
    simdata.buildup = buildup; % Use n-order polynomial builup during movement- 3 good
else
    simdata.buildup = 3;
end
    
simdata.r = 10^-5;
simdata.p = 1;
simdata.gamma = gamma;
simdata.loads = loads;

% Initial and via-point states
xinit = [0 0 0 0 0 0]';
xvia = [0 .10 0 0 0 0]';

simout = adaptiveLQG(xinit,xfinal,xvia,simdata);

% Plot Routine
% -------------------------------------------------------------------------
% figure
subplot(221)
plot(simout.state(1,:),simout.state(2,:));
axis square, axis([-.1 .1 -.02 .18]), hold on;
plot(xinit(1),xinit(2),'ro','LineWidth',2);
plot(xfinal(1),xfinal(2),'ro','MarkerSize',10,'LineWidth',2);
xlabel('x [m]','FontSize',14);
ylabel('y [m]','FontSize',14);

if simdata.tvia>0
    plot(xvia(1),xvia(2),'bo','MarkerSize',15,'LineWidth',2);
end

if norm(simdata.gamma) == 0
    s = ':';
else s = '';
end

subplot(222)
plot(simout.state(1:2,:)',s);
xlabel('Time [#sample]','FontSize',14) 
ylabel('x and y [m]','FontSize',14) 
hold on

%extracting the estimated load parameter
nStep = size(simout.AestCont,3);
LoadEst = zeros(1,nStep);
LoadTrue = zeros(1,nStep);

in = [3,4];
normError = [];

for i = 1:nStep
    
    if ~isnan(simout.AestCont(in(1),in(2),i))
        LoadEst(i) = simout.AestCont(in(1),in(2),i);
        normError = [normError,norm(simout.AestCont(:,:,i)-A)];
    end
    
    LoadTrue(i) = A(in(1),in(2));
    
end

subplot(223)
plot(LoadEst), hold on
plot(LoadTrue,'r');
axis([0 size(simout.AestCont,3) -abs(max(L/m,1))*1.1 abs(max(L/m,1))*1.1])
xlabel('Time [#sample]','FontSize',14) 
ylabel('FF Parameter [Nsm^{-1}]','FontSize',14) 

subplot(224)
plot(normError)
xlabel('Time [#sample]','FontSize',14) 
ylabel('Norm of Error','FontSize',14) 
hold on