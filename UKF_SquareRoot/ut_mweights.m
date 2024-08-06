%UT_MWEIGHTS - Generate matrix form unscented transformation weights
%
% Syntax:
%   [WM,W,c] = ut_mweights(n,alpha,beta,kappa)
%
% In:
%   n     - Dimensionality of random variable
%   alpha - Transformation parameter  (optional, default 0.5)
%   beta  - Transformation parameter  (optional, default 2)
%   kappa - Transformation parameter  (optional, default 3-size(X,1))
%
% Out:
%   WM - Weight vector for mean calculation
%    W - Weight matrix for covariance calculation
%    c - Scaling constant
%
% Description:
%   Computes matrix form unscented transformation weights.

function [WM,WC,W,c] = ut_mweights(n,alpha,beta,kappa)

%   [WM,WC,c] = ut_weights(n,alpha,beta,kappa);
   [WM,WC,c] = ut_weights_2(n,alpha,beta,kappa);

  W = eye(length(WC)) - repmat(WM,1,length(WM));
  W = W * diag(WC) * W';
