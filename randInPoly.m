function [X] = randInPoly(vert,n)

% create random cooridnates inside a convex polygon
% if 3 argout -> random number between (0;2*pi[

% VARGIN: vert = ordered vertex of polygon [x1,x2,...xn;y1,y2,...yn]
%            n = number of point to be created (1 default)

if nargin == 1
    n = 1;
end

if n ~= 1
    error('Distribution of more than 1 point is not random')
end


n_vert = size(vert,2);       % number of vertices in polygon
p = vert';
n_tria = n_vert - 2;          % number of triangles

a = zeros(n_tria,1);
for adx = 1:n_tria
    a(adx,1) = abs( det( [ p(adx+1,:)-p(1,:); p(adx+2,:)-p(1,:) ] ) ) / 2;      % area of each triangle
end
sa = sum(a);
%norm_a = cumsum(a)./sa;
% a23 = abs(det([p2-p1;p3-p1])); % Twice the triangles' areas
% a34 = abs(det([p3-p1;p4-p1]));
% a45 = abs(det([p4-p1;p5-p1]));
% sa = a23+a34+a45;
a_norm = a./sa; % Normalize cumulative areas
r = rand(n,1); % Use this to select the triangle
t_idx = find( cumsum(a_norm)>r, 1 , 'first' );

s = sqrt(rand(n,1)); s2 = [s,s]; % Use these to select
t = rand(n,1); t2 = [t,t]; % points within a triangle

if n==1   
    px = (1-s)*p(1,:) + s.*(1-t)*p(t_idx+1,:) + s.*t*p(t_idx+2,:);
    % px = (1-s)*p(1,:) + s2.*((1-t2).*p(2,:) + t2.*p(3,:));
% elseif n_vert == 4
%     
%     px = (1-s)*p1 + s2.*((1-t2).*pa + t2.*pb);
% elseif n_vert == 5
%     pa = (r<=a)*p2 + ((a<r)&(r<=b))*p3 + (b<r)*p4;      % Generalized vertices
%     pb = (r<=a)*p3 + ((a<r)&(r<=b))*p4 + (b<r)*p5;
%     px = (1-s)*p1 + s2.*((1-t2).*pa + t2.*pb);           % The random points
else
    error('Max number of point supported is 1...')
end
        
% c = [p;p(1,:)]; % Plot the pentagon in red
% plot(c(:,1),c(:,2),'ro',c(:,1),c(:,2),'r-',...
%      px(:,1),px(:,2),'b.') % Plot random points as blue dots
% axis equal

x = px(:,1); y = px(:,2);
X = [x;y];

end