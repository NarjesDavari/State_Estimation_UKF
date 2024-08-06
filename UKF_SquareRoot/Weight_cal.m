function [sigma,W] = Weight_cal (W0,n,alpha,beta)
%W(1,1) = W0;
% for i=2:n+1
%     W(2:n+2,1) = (1 - W(1,1)) / (n + 1);
% end
%% 
W(1,1) = W0;
W(2:3,1)=(1-W0)/2^(-n);
for i=4:n+2
      W(i,1)=W(2,1)/2^(i-2);
end
%% 


sigma = zeros(n,n+2);
sigma(1,1) = 0;
sigma(1,2) = -1 / sqrt(2 * W(2,1));
sigma(1,3) = 1 / sqrt(2 * W(2,1));


for j=2:n
    for i=1:n+2
%         if (i-1)>(j+1)
%            sigma (j,i)=0; 
        if (i-1)==(j+1)
            sigma (1:j,i) = [zeros(j-1,1);j/sqrt(j*(j+1)*W(2,1))];
        elseif (i-1)==0
            sigma (1:j,i) = [sigma(1:j-1,1) ; 0];
        elseif (i-1)<=j
            sigma (1:j,i) = [sigma(1:j-1,i); -1/sqrt(j*(j+1)*W(2,1))];
        end
    end
end

%% 

% WM (1,1) = (1-W0)/alpha^2 +1;
% WM (2:n+2,1) = (1 - W(1,1)) / (n + 1)/alpha^2;
% 
% WC (1,1) = (1-W0)/alpha^2 +2 +beta -alpha^2;
% WC (2:n+2,1) = (1 - W(1,1)) / (n + 1)/alpha^2;

%  W2 = eye(length(WC)) - repmat(WM,1,length(WM));
%  W2 = W2 * diag(WC) * W2';

