function dout = adaptiveLQG(xinit,xfinal,xvia,simdata)

% DOUT = ADAPTIVELQG(XINIT,XFINAL,XVIA,SIMDATA)
%
%   XINI: Initial condition for the state vector
%   XFINAL: Target state
%   XVIA: Coordinate of the via point
%   SIMDATA: Structure with parameters for the simulation (check
%   adaptiveReaching.m)
%
%   DOUT: Output data with simulation parameters

ACont = simdata.A;                   % Model Matrices
BCont = simdata.B;
ANullCont = simdata.ANull;
AestCont = simdata.AestCont;         % Estimated Model
BestCont = simdata.BestCont;
time = simdata.time;                 % Time free and stabilization time   
stab = simdata.stab;
delta = simdata.delta;               % Integration step
alpha = simdata.alpha;               % Cost Parameter [position(x,y) - speed(x,y) - force(x,y)]
r = simdata.r;                       % Cost parameter for control action
p = simdata.p;                       % Number of simulation runs
gamma = simdata.gamma;               % Learning rates for A and B
m = simdata.m;

nstate = size(ACont,1);
ncontr = size(BCont,2);
nStep = round((time+stab)/delta)-1;
nMove = round(time/delta);
Q = zeros(3*nstate,3*nstate,nStep+1);
R = zeros(ncontr,ncontr,nStep);

% Transform continuous time representation into discrete time for actual and
% estimated matrices
A = [ACont,zeros(nstate,2*nstate);zeros(2*nstate,3*nstate)];
A = eye(size(A))+delta*A;

ANull = [ANullCont,zeros(nstate,2*nstate);zeros(2*nstate,3*nstate)];
ANull = eye(size(ANull))+delta*ANull;

B = delta*[BCont;zeros(2*nstate,ncontr)];

Aest = [AestCont,zeros(nstate,2*nstate);zeros(2*nstate,3*nstate)];
Aest = eye(size(Aest))+delta*Aest;
Best = delta*[BestCont;zeros(2*nstate,ncontr)];

% Cost Parameters ---------------------------------------------------------
% Filling in the cost matrices: Q and R - 
% Final Target
xg0 = eye(nstate);
xg = [xg0;-xg0;0*xg0];
for ii = nMove+1:nStep+1
    for i = 1:size(alpha,2)
        Q(:,:,ii) = Q(:,:,ii)+ alpha(i)*xg(:,i)*xg(:,i)';
    end
end

% Uses polynomial buildup for cost matrices during movement
if simdata.buildup > 0
    for ii = 1:nMove
        Q(:,:,ii) = (ii/nMove)^simdata.buildup*Q(:,:,end);
    end
end

%Via point - uses the same alpha-parameter for the via point
xg = [xg0;0*xg0;-xg0];
tvia = simdata.tvia;
if tvia >0
    tvia = round(simdata.tvia/delta);
    for i = 1:size(alpha,2)
        Q(:,:,tvia) = Q(:,:,tvia)+ alpha(i)*xg(:,i)*xg(:,i)';
    end
end

% Cost of Control
for i = 1:nStep
    R(:,:,i) = r*eye(ncontr);
end

% Compute LQG controller---------------------------------------------------
x0 = [xinit;xfinal;xvia];
oZeta = B*B';
[L,~] = basicLQG(Aest,Best,Q,R,x0,oZeta);

for i = 1:p

    % initializing
    xall = zeros(3*nstate,nStep,p);
    xallest = zeros(3*nstate,nStep,p);
    call = zeros(ncontr,nStep,p);
    AestAll = zeros(nstate,nstate,nStep); % Single Simulation run for the moment
    BestAll = zeros(nstate,ncontr,nStep);
    
    currentState = x0;
    currentEstimate = x0;
        
    for t = 1:nStep
        
        % Control
        u = -L(:,:,1)*currentState;

        if simdata.loads(3)>0
            Q = simdata.loads(3)*Q;%Q = 0.95*Q;% .98 or 1.03;
        end
        
        [L,~] = basicLQG(Aest,Best,Q(:,:,t+1:end),R(:,:,t+1:nStep),currentEstimate,zeros(size(Aest)));

        % Turn off the force filed at via-point experiment
        if tvia > 0 && t == round(tvia)
            A = ANull;
        end
        
        % Update of state and estimated state
        nextState = A*currentState + B*u + mvnrnd(zeros(size(A,1),1),oZeta)';
        nextEstimate = Aest*currentState + Best*u;
    
        eps1 = nextState(1:nstate)-nextEstimate(1:nstate);
        
        % Updating Model Matrices
        theta_t = [Aest(3,4)]';
        psy = zeros(1,nstate);
        psy(1,3) = nextState(4);
        theta_up = theta_t + gamma(1)*psy*eps1;
        Aest(3,4) = theta_up(1);
        
        AestCont = (Aest(1:6,1:6)-eye(6))/delta;

        % Updating variables
        currentState = nextState;
        currentEstimate = nextEstimate;
        
        % filling in output vectors
        xall(:,t,i) = currentState;
        xallest(:,t,i) = currentEstimate;
        call(:,t,i) = u;

        AestAll(:,:,t) = AestCont;
        BestAll(:,:,t) = BestCont;
        
    end
    
end

% output simulation resutls
dout.state = xall;
dout.FBgains = L;
dout.estimate = xallest;
dout.control = call;
dout.AestCont = AestAll;
dout.BestCont = BestAll;








