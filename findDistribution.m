function [ rho ] = findDistribution( poses,arena )

N = size(poses,2);
rho = zeros(arena.cellNumber,1);

for nn = 1:N
    ci = 0;
    agentisincell = 0;
    while ~ci 
        agentisincell = agentisincell + 1;
        if agentisincell > size(arena.grid,2); 
            poses(:,nn)
            error('No cell found!'); 
        end          
        ci = inpolygon(poses(1,nn),poses(2,nn),arena.grid{agentisincell}(1,:),arena.grid{agentisincell}(2,:));         
    end  
    rho(agentisincell) = rho(agentisincell) + 1;


end

rho = rho./sum(rho);