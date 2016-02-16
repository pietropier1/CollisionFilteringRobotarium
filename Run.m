% Main robot simulation runner

% In order to use Robotarium, set below switch key to 'experiment'. 
% Both cases in the below switch call the same robotArena.m file which starts the actual simulation. 
% In robotArena.m experiement/simulation cases are automatically detected and no user input is requried

clc; close all; clear classes;
arena.makemovie = false;                                                    % Record matlab plot in video 
arena.ggp = 0.06;                                                           % Radius of robots protected area
arena.dt = 0.05;                                                            % Simulation time step    
arena.agSel = 3;                                                            % Select which agent to follow

% ========= Build Domain Grid =============
[arena,InitConf] = buildArena('3track',arena);                              % change first argument to change grid shape                        

% ====== Compute Markov Transition Matrix =============
%arena.M = initHMM(arena);                                                  % Markov M used in track (overwrite arena.M from buildArena function).          

% ========== Select Type of Analysis =============
switch 'simulation'                                                         % Use 'simulation' or 'experiment'
    case 'simulation'                                                       % Use this when running MATLAB simulation
        arena.N = max([InitConf{:}]);                                       % Number of agents
        
        % ==== Initialize virtual robots ======  
        colliders = 1;
        while ~isempty(colliders)                                           % avoid collision at initial time
            for cellID = 1:arena.cellNumber
                if ~isempty( InitConf{cellID} )
                    for kID = InitConf{cellID}
                        khepera(kID) = Khepera(kID,cellID,arena);                 % Physical agent
                    end
                end
            end
            data = [khepera.myState];
        colliders = collisionFinder( data(1:2,:) , arena.ggp );
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





% ========= P coefficints
% arena.Grid{1} = dilationPoly(arena.grid{1},1.2*arena.ggp); (buildArena)
% [colliders,collided] = ind2sub([N,N],find(DD <= 2*rPZ*1.1)); (collisionFinder)
% new_goal = [agent.myState(1) - 0.01*cos(agent.myState(3)+rdn); (agent/alternaive goal)
% goal(:,idx) = [agent(setColl(idx,1)).myState(1) - 1.2*arena.ggp*cos(agent(setColl(idx,1)).myState(3) + rdn); (robotArena / evaluateCollison)
% distanceTollerance = agent.rcoll .* 0.9;  (agent get.distanceTollerance)
%
