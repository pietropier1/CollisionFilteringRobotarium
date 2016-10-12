function [colliders,collided,collision] = collisionFinder( data , rPZ )
% Compute distance between agents and find index of those involved in collision
% Compute collision clusters

N = size(data,2);
DD = pdist2(data',data');                   % euclidian distance which between each point in data
DD = DD + diag(1e2*ones(1,N));              % filter diagonal elements adding dummy values
[colliders,collided] = ind2sub([N,N],find(DD <= 2*rPZ*1.05));           % couples of agents whose ditance is below robot diameter

DD_coll = DD <= 2*rPZ*1.05;                                             % Binary collision matrix: ij = 1, if collision between i and j 
[idxColl1, idxColl2] = ind2sub([N,N],find(tril(DD_coll) == 1));         
couples = [idxColl1,idxColl2];                                          % Couples involved in the collision
collision = cell(size(couples,1),1);                                    % Clusters of collisions

if ~isempty(couples)                                                    % If there is some collision
    ncoll = 1;                                                          % Initialize clusters to 1
    collision{ncoll} = couples(1,:);                                    % Initialize first cluster = to first couple

    for l = 2:size(couples,1)                                           % for each couple
        new = couples(l,:);                                             % new couple to assign to a cluster
        kk = 0;                                                         % index that runs clusters
        while ~isempty(new)                                             % unitl new cuople has not been assigned
            kk = kk + 1;                                                
            if kk > ncoll                                               % if all existing clusters have been checked
                ncoll = ncoll + 1;                                      % add a cluster
                collision{ncoll} = new;                                 % assign couple to new cluster
                new = [];                                               % cancel new couple
            elseif sum(new(1) == collision{kk});                        % if first agent in couple is part of the cluster, add the second
                collision{kk}(end+1) = new(2); 
                new = [];
            elseif sum(new(2) == collision{kk});                        % if second agent in couple is part of the cluster, add the first
                collision{kk}(end+1) = new(1); 
                new = []; 
            end 
        end
    end
end
whosempty = cellfun('isempty',collision);                               % remove empty cell
collision(whosempty) = []; 

end

