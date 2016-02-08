function [ rho ] = findDistribution( poses,arena )

% Compute swarm distribution over the domain.
% Input:  robot poses -> [x1,x2,...,xN;
%                         y1,y2,...,yN]
%         arena       -> arena struct
% Output: rho         -> fraction of agent in each cell (column vector) 


N = size(poses,2);
rho = zeros(arena.cellNumber,1);
%cellOccupiers = cell(arena.cellNumber,1);                       % ID of agent occuping each cell

for nn = 1:N
    ci = 0;
    agentisincell = 0;
    while ~ci 
        agentisincell = agentisincell + 1;
        if agentisincell > size(arena.grid,2); 
            error('No cell found for one or more agents!'); 
        end          
        ci = inpolygon(poses(1,nn),poses(2,nn),arena.grid{agentisincell}(1,:),arena.grid{agentisincell}(2,:));         
    end 
    %cellOccupiers{agentisincell}(end+1) = nn;
    rho(agentisincell) = rho(agentisincell) + 1;


end

rho = rho./sum(rho);            % normalize distribution