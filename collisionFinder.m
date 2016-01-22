function [colliders,collided] = collisionFinder( data , rPZ )
% Compute distance between agents and find index of those involved in collision
N = size(data,2);
DD = pdist2(data',data');                   % euclidian distance which between each point in data
DD = DD + diag(1e2*ones(1,N));              % filter diagonal elements adding dummy values
[colliders,collided] = ind2sub([N,N],find(DD <= 2*rPZ*1.1)); % couples of agents whose ditance is below robot diameter

end

