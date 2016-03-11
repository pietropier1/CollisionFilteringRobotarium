%function new_goal = deconOptimal(x,xg,r)

close all, clear all; clc
x = [2.25 2.55 2.7 3; 1.8 2.2 2.68 2];


if size(x,1)>2; x = x(1:2,:); end

N = size(x,2);                      % number of agents
mu_0 = eye(N,N);
doPlot = 1;

if doPlot
    r = 0.0565;
    xg = 4*rand(2,N);
    r = 0.25;
    figure(1), hold on, axis([0 4 0 4]), axis equal
    p = repmat(linspace(0,2*pi,25),1,1)';        % circle parameter
    xState = x(1,:);
    yState = x(2,:);
    for ii = 1:N
        % create patch for collision marks
        cclr = [ rand    0.447+rand/2    0.7410];
        pPL = patch(repmat(xState(ii),25,1) + r.*cos(p),repmat(yState(ii),25,1) + r.*sin(p),'k');
        set(pPL,'facecolor',cclr, 'edgecolor','none',...
            'linewidth',2);
        plot( xg(1,ii) , xg(2,ii) ,'s' ,'markerfacecolor' ,cclr, 'markersize',10 ,'markeredgecolor','none')
        text(xg(1,ii)+0.05,xg(2,ii)+0.05,num2str(ii))
    end
end

% First method to compute distance
distance = zeros(N,1);
for ii = 1:N
    for jj = 1:N
        distance(ii,jj) = sqrt((x(1,ii)-xg(1,jj)).^2 + (x(2,ii)-xg(2,jj)).^2);
    end
end

% % Second method to compute distance
% XX = sum(x(1:2,:)'.^2, 2);
% XC = x(1:2,:)' * xg;
% CC = sum(xg'.^2, 2)';
% distance2 = sqrt(bsxfun(@plus, CC, bsxfun(@minus, XX, 2*XC)));


tot_dist0 = sum(diag(distance*mu_0));
% solve the problem for binary variable mu(ij) --> agent i is assigned to j
switch 'mtlb'
    case 'mtlb'
        obj = distance(:);
        intcon = 1:N*N;
        A = -eye(N*N , N*N);
        b = zeros(N*N , 1);
        Aeq = zeros(2*N,N*N);
        beq = ones(2*N,1);
        
        for ii = 1:N
            Aeq(ii , N*(ii-1)+1:N*ii) = ones(1,N);
            Aeq(ii+N , ii:N:end ) = ones(1,N);
        end
        
%         opts = optimoptions('intlinprog','Display','iter','PlotFcns',@optimplotmilp,'NodeSelection','minobj');
        opts = optimoptions('intlinprog','Display','iter','NodeSelection','minobj');
        [solution,fval] = intlinprog(obj,intcon,A,b,Aeq,beq,[],[],opts);
        if isempty(solution) % If the problem is infeasible or you stopped early with no solution
            disp('intlinprog did not return a solution.')
            return % Stop the script because there is nothing to examine
        end
        
        if tot_dist0 < fval
            disp('Optimal Cost greater then Initial')
            x,xg
            pause
        end
        
        mu_val = reshape(solution,N,N);

end

[idx , ~] = ind2sub([N,N],find(mu_val' == 1));   %[goal , agents]
new_goal = xg(:,idx);                            % optimal reordered goals
if doPlot
    figure(1)
    plot( [x(1,:); new_goal(1,:)] , [x(2,:); new_goal(2,:)] ,':k')
end

% Verify Intersection
error('to be cont...Verify that goal assignment is free from collision. If not, produce new goals')
for ag = 1:N
    nlcu = setdiff( 1:N , ag );
    [conf,opp] = conflictHeadings(x(:,ag),new_goal(:,ag),x(:,nclu),new_goal(:,nclu),r);         % check conflict with current goal assignment
    if conf ~= 0; display(['Agent ',num2str(ag),' in conflict with agent',num2str()]); end


end








