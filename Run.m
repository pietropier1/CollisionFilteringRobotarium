% Main robot simulation runner

% Think about adding a GUI for parameters control


clc; close all; clear classes;
makemovie = false;                                                          % Record matlab plot in video 
arena.ggp = 0.06;                                                           % Radius of robots protected area
arena.dt = 0.05;                                                            % Simulation time step    

% ========= Build Domain Grid =============
[arena,InitConf] = buildArena('3track',arena);                              % change first argument to change grid shape                        

% ========= Compute cell areas =============
arena.Dlt = zeros(1,arena.cellNumber);                                      
for cc = 1:arena.cellNumber
    arena.Dlt(cc) = polyarea(arena.grid{cc}(1,:)',arena.grid{cc}(2,:)');
end

% ====== Compute Markov Transition Matrix =============
% uncomment when transitions between cells are desired
%arena.M = initHMM(arena);                                                  % Markov M used in track (overwrite arena.M from buildArena function).          

arena.agSel = 1;                                                            % Select which agent's filter is plot
arena.makemovie = makemovie;

% ========== Select Type of Analysis =============
switch 'simulation'                                                         % Use 'simulation' or 'experiment'
    case 'simulation'                                                       % Use this when running MATLAB simulation
        arena.N = max([InitConf{:}]);                                       % Number of agents
        
        % ==== Initialize virtual robots ======                         
        for cell = 1:arena.cellNumber
            if ~isempty( InitConf{cell} )
                for kID = InitConf{cell}
                    khepera(kID) = Khepera(kID,cell,arena);                 % Physical agent
                end
            end
        end
        [ arena.rho ] = findDistribution( [khepera.myState],arena );        % Agent distribution: fraction of agents in each cell
        
        if size(arena.M) ~= arena.cellNumber; error('Error MM size!'); end  % Check MM size
        optitrackClient = OptitrackSimulator();                             % Create fake stream machine
        pause(0.001)
        % ========= Run Simulation =============
        robotArena(arena,optitrackClient,khepera);                      
        
        
    case 'experiment'                                                       % Use this when running experiments

        %====== Robotarium Initialization ======== 
        r = robotariumMatlabAPI('coverage', 6);
        display(1)
        r.setSimulationMode(false);                                         % Choose simulation mode

        % Get list of available robots
        robots = r.getAvailableRobots();
        arena.N = length(robots);                                           % Total number of robots
        r.setDeltaDiskDistance(10.0);                                       % Set neighbor disk
        %Get initial robot poses
        poses = r.getPoses();
        [ arena.rho ] = findDistribution( poses,arena );                    % Agent distribution: fraction of agents in each cell          

        if size(arena.M) ~= arena.cellNumber; error('Error MM size!'); end
        pause(0.001)
        % ========= Run Simulation =============
        robotArena(arena,r);
        
end