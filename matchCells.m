function [ val ] = matchCells( cluster , listOfClusters )
% Return 1 if CLUSTER is present in LISTOFCLUSTERS ( not necessarly in the same order )

val = 0;                                                                    % initialize result to 0
cluster = sort(cluster);                                                    % sort elements in the cluster

matchingLength = cellfun(@(x) find(length(x)==size(cluster,2)),listOfClusters, 'UniformOutput', false); % find clusters of matching length
empty = cellfun('isempty',matchingLength);                               % index of not-matching clusters


for cc = find(~empty)'                                                      % for those clusters of matching length
    clu = listOfClusters{cc};                                   
    if sum(sort(clu) == cluster) == size(clu,2);                            % check if cluster is the same
        val = 1;
        return
    end

end