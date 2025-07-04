function EKFLocalisation
close all; clear all;
global xTrue;global Map;global QTrue;global PYTrue;global nSteps;

nSteps = 6000;

Map = 140*rand(2,30)-70;

% True covariance of errors used for simulating robot movements
QTrue = diag([0.01,0.01,1*pi/180]).^2;
PYTrue = diag([2.0,3*pi/180]).^2;

% Modeled errors used in the Kalman filter process
QEst = eye(3,3)*QTrue;
PYEst = eye(2,2)*PYTrue;

xTrue = [1;-40;-pi/2];
xOdomLast = GetOdometry(1);

%initial conditions:
xEst =xTrue;
PEst = diag([1,1,(1*pi/180)^2]);

%%%%%%%%%  storage  %%%%%%%%
InnovStore = NaN*zeros(2,nSteps);
SStore = NaN*zeros(2,nSteps);
PStore = NaN*zeros(3,nSteps);
XStore = NaN*zeros(3,nSteps);
XErrStore = NaN*zeros(3,nSteps);

%initial graphics
figure(1); hold on; grid off; axis equal;
plot(Map(1,:),Map(2,:),'g*');hold on;
set(gcf,'doublebuffer','on');
hObsLine = line([0,0],[0,0]);
set(hObsLine,'linestyle',':');

for k = 2:nSteps
    
    %do world iteration
    SimulateWorld(k);
    
    %figure out control by subtracting current and previous odom values
    xOdomNow = GetOdometry(k);
    u = tcomp(tinv(xOdomLast),xOdomNow);
    xOdomLast = xOdomNow;
    
    %do prediction
    xPred = tcomp(xEst,u); % function f
    xPred(3) = AngleWrap(xPred(3));
    PPred = A(xEst,u)* PEst *A(xEst,u)' + B(xEst,u)* QEst * B(xEst,u)';
        
    %observe a randomn feature
    [z,iFeature] = GetObservation(k);
        
    if(~isempty(z))
        %predict observation
        zPred = DoObservationModel(xPred,iFeature,Map);
        
        % get observation Jacobian
        H = GetObsJac(xPred,iFeature,Map);
        
        %do Kalman update:
        Innov = z-zPred;
        Innov(2) = AngleWrap(Innov(2));
        
        S = H*PPred*H'+PYEst;
        W = PPred*H'*inv(S);
        
        xEst = xPred+ W*Innov;
        xEst(3) = AngleWrap(xEst(3));

        PEst = PPred - W*H*PPred; 
        PEst = 0.5*(PEst+PEst'); % ensure symetry
        
    else
        %There was no observation available
        xEst = xPred;
        PEst = PPred;
        Innov = [NaN;NaN];
        S = NaN*eye(2);
    end;
            
    % plot every 200 updates
    if(mod(k-2,200)==0)
        DoVehicleGraphics(xEst,PEst(1:2,1:2),8,[0,1]);        
        if(~isempty(z))
            set(hObsLine,'XData',[xEst(1),Map(1,iFeature)]);
            set(hObsLine,'YData',[xEst(2),Map(2,iFeature)]);
        end;
        drawnow;        
    end;            
    
    %store results:
    InnovStore(:,k) = Innov;
    PStore(:,k) = sqrt(diag(PEst));
    SStore(:,k) = sqrt(diag(S));
    XStore(:,k) = xEst;
    XErrStore(:,k) = xTrue-xEst;
end;

DoGraphs(InnovStore,PStore,SStore,XStore,XErrStore);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function DoGraphs(InnovStore,PStore,SStore,XStore,XErrStore)

figure(1); print -depsc 'EKFLocation.eps'

figure(2);
subplot(2,1,1);plot(InnovStore(1,:));hold on;plot(SStore(1,:),'r');plot(-SStore(1,:),'r')
title('Innovation');ylabel('range');
subplot(2,1,2);plot(InnovStore(2,:)*180/pi);hold on;plot(SStore(2,:)*180/pi,'r');plot(-SStore(2,:)*180/pi,'r')
ylabel('Bearing (deg)');xlabel('time');
print -depsc 'EKFLocationInnov.eps'

figure(2);
subplot(3,1,1);plot(XErrStore(1,:));hold on;plot(3*PStore(1,:),'r');plot(-3*PStore(1,:),'r');
title('Covariance and Error');ylabel('x');
subplot(3,1,2);plot(XErrStore(2,:));hold on;plot(3*PStore(2,:),'r');plot(-3*PStore(2,:),'r')
ylabel('y');
subplot(3,1,3);plot(XErrStore(3,:)*180/pi);hold on;plot(3*PStore(3,:)*180/pi,'r');plot(-3*PStore(3,:)*180/pi,'r')
ylabel('\theta');xlabel('time');
print -depsc 'EKFLocationErr.eps'

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [z,iFeature] = GetObservation(k)
global Map;global xTrue;global PYTrue;global nSteps;
%fake sensor failure
if(abs(k-nSteps/2)<0.1*nSteps)
z = [];
    iFeature = -1;
else
    iFeature = ceil(size(Map,2)*rand(1));
    z = DoObservationModel(xTrue, iFeature,Map)+sqrt(PYTrue)*randn(2,1);
    z(2) = AngleWrap(z(2));
end;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [z] = DoObservationModel(xVeh, iFeature,Map)
Delta = Map(1:2,iFeature)-xVeh(1:2);
z = [norm(Delta);
    atan2(Delta(2),Delta(1))-xVeh(3)];
z(2) = AngleWrap(z(2));


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function SimulateWorld(k)
global xTrue;
u = GetRobotControl(k);
xTrue = tcomp(xTrue,u);
xTrue(3) = AngleWrap(xTrue(3));


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [xnow] = GetOdometry(k)
persistent LastOdom; %internal to robot low-level controller
global QTrue;
global xTrue;
if(isempty(LastOdom))
    LastOdom = xTrue;
end;
u = GetRobotControl(k);
xnow =tcomp(LastOdom,u);
uNoise = sqrt(QTrue)*randn(3,1);
xnow = tcomp(xnow,uNoise);
LastOdom = xnow;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% construct a series of odometry measurements
function u = GetRobotControl(k)
global nSteps;
u = [0; 0.025 ; 0.1*pi/180*sin(3*pi*k/nSteps)];
%u = [0; 0.15 ; 0.3*pi/180];




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Functions to be completed

% h(x) Jacobian
function jH = GetObsJac(xPred, iFeature,Map)
jH = zeros(2,3);
Delta = (Map(1:2,iFeature)-xPred(1:2));
r = norm(Delta);
jH(1,1) = -Delta(1) / r;
jH(1,2) = -Delta(2) / r;
jH(2,1) = Delta(2) / (r^2);
jH(2,2) = -Delta(1) / (r^2);
jH(2,3) = -1;

% f(x,u) Jacobian % x
function Jac  = A(x,u)
s1 = sin(x(3));
c1 = cos(x(3));
Jac  = [1 0 -u(1)*s1-u(2)*c1;
        0 1 u(1)*c1-u(2)*s1;
0 0 1];

% f(x,u) Jacobian % u
function Jac  = B(x,u)
s1 = sin(x(3));
c1 = cos(x(3));
Jac =[c1 -s1 0; 
    s1 c1 0;
    0 0 1];