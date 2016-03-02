function decon


close all, clear all, clc


x = [2.25 2.55 2.7 3; 1.8 2.2 2.68 2; 2 0.3 0.4 0.6];
N = size(x,2);
xg = 4*rand(2,N);

r = 0.25;



figure(), hold on, axis([0 4 0 4]), axis equal
p = repmat(linspace(0,2*pi,25),1,1)';        % circle parameter
xState = x(1,:);
yState = x(2,:);
tState = x(3,:);

for ii = 1:N
% create patch for collision marks
cclr = [ rand    0.447+rand/2    0.7410];
pPL = patch(repmat(xState(ii),25,1) + r.*cos(p),repmat(yState(ii),25,1) + r.*sin(p),'k');
set(pPL,'facecolor',cclr, 'edgecolor','none',...
    'linewidth',2);
plot( xg(1,ii) , xg(2,ii) ,'s' ,'markerfacecolor' ,cclr, 'markersize',10 ,'markeredgecolor','none')
text(xg(1,ii)+0.05,xg(2,ii)+0.05,num2str(ii))

end

% center of cluster CoC

xCC = [sum(xState)/N ; sum(yState)/N];
plot(xCC(1) , xCC(2) , 'ok', 'markersize',8,'markerfacecolor','k')

distance = zeros(N,1);
for ii = 1:N
    for jj = 1:N
        %distance(ii,1) = distance(ii,1) + norm( (x(1:2,ii) - xg(1:2,jj)) , 2 ).* mu(jj);
        distance(ii,jj) = sqrt((x(1,ii)-xg(1,jj)).^2 + (x(2,ii)-xg(2,jj)).^2);
    end
end

mu = binvar(N,N);
obj = sum( diag(distance*mu') );
const = [sum(mu,1) == ones(1,N),...
         sum(mu,2) == ones(N,1)];

%ops = sdpsettings('solver','mosek');
ops = sdpsettings('solver','bnb');

optimize(const,obj,ops)
mu_val = value(mu)
[idx , jdx] = ind2sub([N,N],find(mu_val == 1));

plot( [x(1,:); xg(1,idx)] , [x(2,:); xg(2,idx)] ,':k')





