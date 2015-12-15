classdef Khepera < handle
    % Physical representation of robots
    
    properties
        ID;
        myState;
        w;              % angular velocity
        v;              % linear velocity
        cell;
    end
    
    properties (Constant)
       radius = 0.05; 
    end
    
    methods % contructor
        function khepera = Khepera(kID,cell,arena)
            khepera.ID = kID;
            khepera.cell = cell;
            [x,y,t] = randInPoly(arena.grid{cell});
            khepera.myState = [x;y;t];
        end
    end
    
    methods    
        function move(khepera,v,w)
            dt = 0.05;
            khepera.v = v;
            khepera.w = w;
            khepera.myState = [khepera.myState(1) + khepera.v.*cos(khepera.myState(3) + khepera.w.*dt)*dt;
                             khepera.myState(2) + khepera.v.*sin(khepera.myState(3) + khepera.w.*dt)*dt;
                             khepera.myState(3) + khepera.w.*dt];
            khepera.myState(3) = wrapTo2Pi(khepera.myState(3));        
        end
        
    end
    
    methods (Access = 'private')
        function position = randomInCell(khepera,arena)
            c = khepera.cell;
            r = khepera.radius;
            position = [arena.grid{c}(1,1)+r + (arena.grid{c}(1,2) - arena.grid{c}(1,1) -2*r)*rand(1,1);
                        arena.grid{c}(2,2)-r + (arena.grid{c}(2,3) - arena.grid{c}(2,2) +2*r)*rand(1,1)];       
        end
        
    end
    
end

