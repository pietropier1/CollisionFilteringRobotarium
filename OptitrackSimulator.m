classdef OptitrackSimulator
    % Simulate the presence of an optitrack client
    
    properties
        data;           % number of bodies
   
    end
    
    methods % constructor
        function optitrackClient = OptitrackSimulator()
            disp('Virtual client created');
            
        end
    end
    
    
    methods
        function outData = OSupdate(optitrackClient,khepera)
           optitrackClient.data = [khepera.myState];
           outData = optitrackClient.data;
            
            
            
        end
    end
    
    
end
