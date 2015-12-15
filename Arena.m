classdef Arena < handle
    
    properties %(SetAccess = private)
        cellNumber = 4;             % x direction # of cell
        cellLength = 1;             % cell size
        cellHeigth = 1;
        Xgrid;
        Ygrid;
        centers;                    % cell centers
        Dlt;                        % cell area
        N;                          % number of agents
        r;                          % cell occupied by agent 1...N
        collisionsCoordinates = []; 
        headings;       % headings to impose on agent when collision is experianced     
        static;
        cellOccup;
        disposition = 'block'; % 'alligned'
    end
    
    methods % Constructor
        function arena = Arena(static)  
            arena.static = static;
            arena.headings = zeros(1,arena.N);
            arena.N = sum(1:arena.cellNumber);      % number of agent
            first_ag = 1;
            last_ag = 1;
            for aa = 1:arena.cellNumber
                arena.cellOccup{aa} = first_ag:last_ag;
                first_ag = last_ag + 1;
                last_ag = last_ag + aa+1;
            end
            arena.Dlt = arena.cellLength*arena.cellHeigth;
            switch arena.disposition
                case 'alligned'
                arena.centers = 1:arena.cellNumber;
                arena.Xgrid = 1-arena.cellLength/2:arena.cellLength:arena.cellLength*arena.cellNumber+arena.cellLength/2;
                arena.Ygrid = arena.cellHeigth/2.*[-1;1;1;-1];
                case 'block'
                    if arena.cellNumber ~= 4; disp('Error: block config only works with 4 cells'); return; end
                
                arena.Xgrid = arena.cellLength.*[-1,0,0,-1;0,1,1,0;0,1,1,0;-1,0,0,-1]; % clockwise ordering of cell nodes starting from top left corner
                arena.Ygrid = arena.cellHeigth.*[1,1,0,0;1,1,0,0;0,0,-1,-1;0,0,-1,-1];
                arena.centers = [arena.cellLength/2.*[-1,1,1,-1];arena.cellHeigth/2.*[1,1,-1,-1]];
            end
        end
    end
    
    methods
        % ---------------    
         
        
        % ---------------
        function updateArena(arena,agent)      
            
            
            arena.collisionsCoordinates = evaluateCollisions(arena,state,agent);   
        end
        
    end
    
    
end
        