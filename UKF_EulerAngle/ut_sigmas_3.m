
function [X,X_aug,X_aug2,W] = ut_sigmas_3(M,P,Q)

A = utchol(P); %%%schol(P);
%   A = chol(P);
B = utchol(Q);
S_aug = blkdiag(A,B);
N_aug = length(S_aug);
w0 = 1-N_aug/3;
 X_aug = sqrt(N_aug/(1-w0))*[zeros(size(S_aug,1)) S_aug' -S_aug'];
 
N_aug2 = length(B);
w0_2 = 1-N_aug2/3;
 X_aug2 = sqrt(N_aug2/(1-w0_2))*[zeros(size(M)) B -B];
  
  N = length(M);
  w0 = 1-N/3;
  X = [zeros(size(M)) A -A];
  X = sqrt(N/(1-w0))*X + repmat(M,1,size(X,2));
  
  W(1,1)=w0;
  W(2:2*size(M)+1,1) = (1-w0)/2/N;
end