%UT_TRANSFORM  Perform unscented transform
%
% Syntax:
%   [mu,S,C,X,Y,w] = UT_TRANSFORM(M,P,g,g_param,tr_param)
%
% In:
%   M - Random variable mean (Nx1 column vector)
%   P - Random variable covariance (NxN pos.def. matrix)
%   g - Transformation function of the form g(x,param) as
%       matrix, inline function, function name or function reference
%   g_param - Parameters of g               (optional, default empty)
%   tr_param - Parameters of the transformation as:       
%       alpha = tr_param{1} - Transformation parameter      (optional)
%       beta  = tr_param{2} - Transformation parameter      (optional)
%       kappa = tr_param{3} - Transformation parameter      (optional)
%       mat   = tr_param{4} - If 1 uses matrix form         (optional, default 0)
%       X     = tr_param{5} - Sigma points of x
%       w     = tr_param{6} - Weights as cell array {mean-weights,cov-weights,c}
%
% Out:
%   mu - Estimated mean of y
%    S - Estimated covariance of y
%    C - Estimated cross-covariance of x and y
%    X - Sigma points of x
%    Y - Sigma points of y
%    w - Weights as cell array {mean-weights,cov-weights,c}
%
% Description:
%   ...
%   For default values of parameters, see UT_WEIGHTS.

function [mu,P,S,Y,C,X] = ut_transform(M , P , g , G_ , Q , Qc , C_ , fb , fb_ , Wib_b , Wib_b_ , gl , dt , WM ,WC , W ,sigma,W2,WM2,WC2,alpha,beta, c,I)

  
  %
  % Calculate sigma points
  %
    
   [X,X_aug,X_aug2]= ut_sigmas(M,P,Q,c);
%     [X,X_aug2] = ut_sigmas_2(length(M),M,P,Q,sigma,alpha);
%%%%%%%%%%%%%%
%    [X,X_aug,X_aug2,W] = ut_sigmas_3(M,P,Q);
%   WM =(W);
%   WC = (W);
  %%%%%%%%%%%%
  %
  % Propagate through the function
  %

  
  %%%%%%
  
    Y = zeros(15,size(X,2));
    for i=1:size(X,2)
       %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%         [X1_dot]        = feval(g , X(:,i) , C_ , fb_ , Wib_b_ , gl, Q ,dt);
%           X1_dot(10:15,1) = X_aug2(10:15,i);
%         delta_X1        =  X1_dot * dt;
%         X_plus_delta_X1 = M + delta_X1;
%         Euler_I         = [X_plus_delta_X1(7),X_plus_delta_X1(8),X_plus_delta_X1(9)] ;
%         Cbn             = InCBN(Euler_I);
%         [X2_dot]        = feval(g , X_plus_delta_X1 , Cbn  , fb  , Wib_b  , gl, Q, dt);
%            X2_dot(10:15,1) = X_aug2(10:15,i);
%         delta_X2        =  X2_dot * dt;
%         delta_X         = (delta_X1 + delta_X2)/2;
%         X_hat           = M + delta_X;       
       %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
       [X1_dot]        = feval(g , X(:,i) , C_ , fb_ , Wib_b_ , gl, Q ,dt );
%        X1_dot(10:15,1) = X_aug(15+7:15+12,50+i);
       X1_dot(10:15,1) = X_aug2(10:15,i);
        delta_X1        =  X1_dot * dt;
        X_hat = M + delta_X1;
       %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        Y(:,i) = X_hat;
    end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  
    mu = Y*WM;
    S  = Y*W*Y';
    C  = X*W*Y';
    P = S;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%     mu = WM(1,1)*Y(:,1) + WM(2,1)*sum(Y(:,2:end),2);%%Y*WM;
%     S2=zeros(15,15);
%     for j=1:size(Y,2)
%         Y_(:,j)=Y(:,j)-mu;
%     end
%      for j=1:size(Y,1)
%         for i = 1: size(Y,2)
%             S2(j,i) = Y_(j,i)*WC(i,1);
%             C2(j,i) =  X(j,i)*WC(i,1);
%         end
%     end
%    P  = S2*Y_';
%    C  = C2*Y_';
%    
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%         a=.1;
%         N =length(M);
%         W0 = 1-N/3; %%1-1/a^2; %%
%         Wj = (1-W0)/2/N; %%1/2/N/a^2; %%%
% %         Y1 = zeros(length(M),2*length(M)+1);
% %         Y1(:,1) = sqrt(abs(W0))*Y(:,1);
% %         Y1(:,2:end) = sqrt(abs(Wj))*Y(:,2:end);
% %         [~,Rs] = qr(Y1(:,2:end)');
% %         S1 = Rs(1:length(P),1:length(P));
% %         P = S1*S1';
% 
%    mu = W0*Y(:,1) + Wj*sum(Y(:,2:end),2);%%Y*WM;
%    Y_(:,1) = sqrt(abs(W0+(1-alpha^2+beta)))* (Y(:,1)-mu);
%    YY = sqrt(Wj)*(Y(:,2:2*N+1)-mu*ones(1,2*N));
%    [~,Rs] = qr(YY');
%    Ss = Rs(1:length(P),1:length(P));
%    Sy = cholupdate(Ss,Y_(:,1));
%    S = Sy;
%    P = Sy*Sy';
  %%%%%%%%%%%%%%%%%%%%%%%%%%% new sigma point based on (n+1) point  
%     mu = W2(1,1)*Y(:,1) + W2(2,1)*sum(Y(:,2:end),2);%%Y*WM2;
%     S2 = zeros(length(M),length(M)+2);
%     C2 = zeros(length(M),length(M)+2);
%     for j=1:size(Y,2)
%         Y_(:,j)=Y(:,j)-mu;
%         X_(:,j)=X(:,j)-M;
%     end
%     for j=1:size(Y,1)
%         for i = 1: size(Y,2)
%             S2(j,i) = Y_(j,i)*W2(i,1);
%             C2(j,i) =  X_(j,i)*W2(i,1);
%         end
%     end
%      P  = S2*Y_';
%      C  = C2*Y_';
%      S = S2;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %Euler Range Checking
    if mu(9) > pi
        mu(9) = mu(9) - 2*pi;
    elseif mu(9) < -pi
        mu(9) = mu(9) + 2*pi;
    else
        %No Operation
    end    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
 
  end

  
