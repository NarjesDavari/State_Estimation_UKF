% randWalk1D.m
% (symmetric) random walk in 1D

n=100000; %number of steps
figure;
hold on;
c = ['r','g','b','c','m','y','k'];
for l=1:100
%x = (rand(n,1) - 0.5*ones(n,1)); %uniformly distributed jumps
%x = randn(n,1); % normal
%jumps are +-1
x = (rand(n,1) - 0.5*ones(n,1));
indP = find(x>0);
indM = find(x<=0);
x(indP) = 1;
x(indM) = -1;
z = zeros(n,1);
for i=2:n
z(i) = z(i-1)+ x(i);
end
plot(z,'Color', c(mod(l,7)+1),'LineWidth',2);
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 
step_num=20;
walk_num=1000;
% function random_walk_2d_simulation (step_num ,walk_num )
time = 0 : step_num;
d2_ave = zeros(step_num+1,1);
d2_max = zeros(step_num+1,1);
%
% Take the walk WALK_NUM times.
%
for walk = 1 : walk_num
x = zeros(step_num+1,1);
y = zeros(step_num+1,1);
for step = 2 : step_num + 1
%
% We are currently at ( X(STEP-1), Y(STEP-1) ).
% Consider the four possible points to step to.
%
destination = [ x(step-1) + 1.0, y(step-1); ...
x(step-1) - 1.0, y(step-1); ...
x(step-1), y(step-1) + 1.0; ...
x(step-1), y(step-1) - 1.0 ];
%
% Choose destination 1, 2, 3 or 4.
%
k = ceil ( 4.0 * rand );
%
% Move there.
%
x(step) = destination(k,1);
y(step) = destination(k,2);
%
% Update the sum of every particle's distance at step J.
%
d2 = x(step)^2 + y(step)^2;
d2_ave(step) = d2_ave(step) + d2;
d2_max(step) = max ( d2_max(step), d2 );
end
end
%
% Average the squared distance at each step over all walks.
%
d2_ave(:,1) = d2_ave(:,1) / walk_num;
%
% Make a plot.
%
clf
plot ( time, d2_ave, time, d2_max, 'LineWidth', 2 );
xlabel ( 'Time' )
ylabel ( 'Distance squared' )
title_string = sprintf ( '2D Random Walk Ave and Max - %d walks, %d steps', walk_num, step_num );
title ( title_string );
% return
% end


%% <font>% setup random walk values
numDimensions=1;
numSteps=100;
randValues = round(rand(numSteps, numDimensions));

% One dimensional random walk
if numDimensions == 1
    randValues(randValues == 0) = -1;
% Two dimensional random walk
elseif numDimensions == 2
end
position = zeros(numSteps+1, numDimensions);
for k = 2:numSteps+1
 position(k,:) = position(k-1,:) + randValues(k-1,:);
 end
