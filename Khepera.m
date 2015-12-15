classdef Khepera < handle
    % Simulator of physical robot
    
    properties
        ID;             % robot ID
        myState;        % robot own state [x;y;theta];
        cell;           % cell occupied
        dt;             % simulation time step
    end
    
    methods % contructor
        function khepera = Khepera(kID,cell,arena)
            khepera.ID = kID;
            khepera.cell = cell;
            X = randInPoly(arena.grid{cell});
            khepera.myState = [X;2*pi*rand(1,1)];
            khepera.dt = arena.dt;
        end
    end
    
    methods    
        function updateDynamics(khepera,v,w)
            khepera.myState = [khepera.myState(1) + v.*cos(khepera.myState(3) + w.*khepera.dt)*khepera.dt;
                               khepera.myState(2) + v.*sin(khepera.myState(3) + w.*khepera.dt)*khepera.dt;
                               khepera.myState(3) + w.*khepera.dt];
            khepera.myState(3) = wrapTo2Pi(khepera.myState(3));        
        end
        
    end
    
end

