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

function [mu,P,Y] = ut_transform(M , S , g , G_ , C_ , fb , fb_ , Wib_b , Wib_b_ , gl , dt , WM ,WC , W ,sigma,W2,alpha, c, Q,I)

  
  %
  % Calculate sigma points
  %
    
   X = ut_sigmas(M,S,c);
%    X = ut_sigmas_2(length(M),M,P,sigma,alpha);
  
  %
  % Propagate through the function
  %

  
  %%%%%%
  
    Y = zeros(15,size(X,2));
    for i=1:size(X,2)
       %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        [X1_dot]        = feval(g , X(:,i) , C_ , fb_ , Wib_b_ , gl );
        delta_X1        =  X1_dot * dt;
        X_plus_delta_X1 = M + delta_X1;
        Euler_I         = [X_plus_delta_X1(7),X_plus_delta_X1(8),X_plus_delta_X1(9)] ;
        Cbn             = InCBN(Euler_I);
        [X2_dot]        = feval(g , X_plus_delta_X1 , Cbn  , fb  , Wib_b  , gl);
        delta_X2        =  X2_dot * dt;
        delta_X         = (delta_X1 + delta_X2)/2;
        X_hat           = M + delta_X;       
       %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%        [X1_dot]        = feval(g , X(:,i) , C_ , fb_ , Wib_b_ , gl );
%         delta_X1        =  X1_dot * dt;
%         X_hat = M + delta_X1;
       %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        Y(:,i) = X_hat;
    end
%  
    mu = Y*WM;
   [q,r] = qr([sqrt(WC(2,1))*(Y(:,2:end) - mu)]'); 
%     S_ = cholupdate(r(1:length(r)/2,:) ,Y(:,1)- mu ,'-');
S_ = r(1:length(r)/2,:);
     P = S_'*S_ + sign(WC(1,1))*(sqrt(WC(1,1))*(Y(:,1)- mu))*(sqrt(WC(1,1))*(Y(:,1)- mu))'; %%% if WC(1,1)=WC0<0
    S = chol(P);
  %%%%%%%%%%%%%%%%%%%%%%%%%%% new sigma point based on (n+1) point  
%     mu = Y*W2;
%     S2 = zeros(length(M),length(M)+2);
%     C2 = zeros(length(M),length(M)+2);
%     for i = 1: size(Y,2)
%         S2(:,i) = Y(:,i)*W2(i,1);
%         C2(:,i) =  X(:,i)*W2(i,1);
%     end
%      S  = S2*Y';
%      C  = C2*Y';
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

  
