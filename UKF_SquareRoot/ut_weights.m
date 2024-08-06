%UT_WEIGHTS - Generate unscented transformation weights
%
% Syntax:
%   [WM,WC,c] = ut_weights(n,alpha,beta,kappa)
%
% In:
%   n     - Dimensionality of random variable
%   alpha - Transformation parameter  (optional, default 0.5)
%   beta  - Transformation parameter  (optional, default 2)
%   kappa - Transformation parameter  (optional, default 3-n)
%
% Out:
%   WM - Weights for mean calculation
%   WC - Weights for covariance calculation
%    c - Scaling constant
%
% Description:
%   Computes unscented transformation weights.

function [WM,WC,c] = ut_weights(n,alpha,beta,kappa)

%
% Apply default values
%
%   alpha = 1e-3;
%   beta  = 2;
%   kappa = 0;
	  
%
% Compute the normal weights 
%
lambda = alpha^2 * (n + kappa) - n;
	  
WM = zeros(2*n+1,1);
WC = zeros(2*n+1,1);
for j=1:2*n+1
  if j==1
    wm = lambda / (n + lambda);
    wc = lambda / (n + lambda) + (1 - alpha^2 + beta);
  else
    wm = 1 / (2 * (n + lambda));
    wc = wm;
  end
  WM(j) = wm;
  WC(j) = wc;
end

c = n + lambda;
