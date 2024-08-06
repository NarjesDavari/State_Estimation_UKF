%UT_SIGMAS - Generate Sigma Points for Unscented Transformation
%
function [X_point,X_aug2] = ut_sigmas_2(n,X,P,Q,sigma,alpha)

% alpha=0.5;
 A = utchol(P); %%%schol(P);
 B = utchol(Q);
 s_aug=blkdiag(A,B);

 for i=1:n+2
     X_point(:,i) = X + alpha*A* sigma (:,i);
     X_aug2(:,i) = X + alpha*B* sigma(:,i);
 end