% Main robot simulation runner
clc; close all; clear classes;
makemovie = true;
arena.ggp = 0.06;                      % radius of robots + protected area

%=========================================
[arena,InitConf] = buildArena('3track',arena);
arena.Dlt = zeros(1,arena.cellNumber);
for cc = 1:arena.cellNumber
    arena.Dlt(cc) = polyarea(arena.grid{cc}(1,:)',arena.grid{cc}(2,:)');
end
%arena.M = initHMM(arena);  % Markov M for track HMM
arena.agSel = 1;
arena.makemovie = makemovie;

switch 'experiment'
    case 'simulation'                    % radius of robots + protected area
        arena.N = max([InitConf{:}]);
        % ==== Initial Khepera robots ======
        arena.rho = zeros(arena.cellNumber,1);
        
        for cell = 1:arena.cellNumber
            if ~isempty( InitConf{cell} )
                for kID = InitConf{cell}
                    khepera(kID) = Khepera(kID,cell,arena);
                end
            end
            arena.rho(cell) = size(InitConf{cell},2);
        end

        arena.rho = arena.rho/arena.N;
        
        if size(arena.M) ~= arena.cellNumber; error('Error MM size!'); end
        optitrackClient = OptitrackSimulator();
        pause(0.001)
        robotArena(arena,optitrackClient,khepera);
        
        
    case 'experiment'

        %====== Robotarium Initialization ======== 
        r = robotariumMatlabAPI('coverage', 6);
        display(1)
        r.setSimulationMode(false);

        % Get list of available robots
        robots = r.getAvailableRobots();
        arena.N = length(robots);
        r.setDeltaDiskDistance(10.0); %set neighbor disk
        %Get initial robot poses
        poses = r.getPoses();
        [ arena.rho ] = findDistribution( poses,arena );

        if size(arena.M) ~= arena.cellNumber; error('Error MM size!'); end
        pause(0.001)
        robotArena(arena,r);
        
end