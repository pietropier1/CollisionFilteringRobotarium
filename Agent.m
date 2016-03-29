classdef Agent < handle
    
    % Class for agent    
    properties (SetAccess = private, GetAccess = public)
        ID = 1;                             % agent ID
        cell = 1;                           % cell agent is at
        selected;                           % is agent selected for tracking? (boolean)
        myState = [0;0;0];                  % agent state = [x;y;th] -> global reference
        grid = {[-1,1,1,-1;1,1,-1,-1]};     % arena grid info [center, corners in cw order from top left]
        Grid;                               % reduced grid for account robot dimension
        Interface;                          % cell interface coordinates
        nAgents;                            % total number of agens
        Dlt;                                % Single cells area
        rcoll;                              % radius of robot area 
        phi = [];                           % N*dlt/Dlt*mu ratio for each cell
        M;                                  % transition probability matrix
        err2goal;                           % error angle to goal
        mu;                                 % swarm projected distribution
        estimate;                           % estimated cell for robot location
        belief;                             % agent position belief
        dt;                                 % simulation time step size
        measureTime = 0;                    % time counter    
        time = 0;                           % simulation running time
        cell_story = [];                    % cell movements progression
        estimateStory = [];                 % estimate progression 

    end
    
    properties (Access = private)
        plotindex;                          % index for continuous plot
        plotHandle;                         % plot handle for agent actual position
        plotHandle2;                        % plot handle for marker
        plotHandle3;                        % plot handle for belief
    end
        
    
    properties (SetAccess = public)
        goal;                               % agent coverage task checkpoint list [xg1,xg2,...;yg1,yg2,...]
        loms = 0;                           % loss of minimum separation flag
        nCollisions = 0;                    % number of collision
        transitioning = 0;                  % cell transfer flag: 1 if agent is transfering / agent going to an interface goal
        waitingTime;                        % number of step to wait before start moving again
    end
    
    properties(Constant)
        wMax = 2;                           % max angular velocity
        lenStory = 50;                      % length of continuous plot
        measureWindow = 220;                 % period of collision registration window
        cellDwell = 90;                    % cell dwell time
        speed = 0.04;                       % linear velocity
        headindsTollerance = 0.4;           % tollerance in headings measure
    end
    
    properties(Dependent)
        dlt;
        distanceTollerance;
    end
      
    % ======= Constructor =========
    methods
        function agent = Agent(ID,arena,data)
            if nargin == 3
                agent.ID = ID;
                agent.cell_story = zeros(agent.lenStory,1);
                agent.estimateStory = zeros(agent.lenStory,1);
                if agent.ID == arena.agSel;agent.selected = true; end           % turn selection flag on
                agent.rcoll = arena.ggp;
                agent.grid = arena.grid;
                agent.Grid = arena.Grid;
                agent.Interface = arena.Interface;
                agent.nAgents = arena.N;
                agent.myState = data;   
                agent.cell = findCell(agent);
                agent.Dlt = arena.Dlt;
                agent.M = arena.M;
                agent.dt = arena.dt;
               
                % Viterbi Initialization
                Ncell = arena.cellNumber;
                agent.mu = 1/Ncell.*ones(Ncell,1);
                agent.phi = (agent.nAgents.*agent.dlt).*inv(diag(agent.Dlt))*agent.mu;
                G = 1-agent.phi';
                agent.belief = zeros(Ncell,1);
                for i=1:Ncell;
                    mui=zeros(Ncell,1); mui(i)=agent.mu(i);
                    agent.belief(i)=G*mui;
                end
                %agent.belief = G .* agent.mu; 
                [~, agent.estimate] = max(agent.belief);
                
                % ====== Assign initial random goal inside current cell ===
                agent.goal = [randInPoly(agent.Grid{agent.cell}),randInPoly(agent.Grid{agent.cell})];
                
                % ======== Create estimation plot =============
                if agent.selected == true; agent.plotindex = 1; 
                    FH = figure();
                    set(FH,'units','normalized','position',[.5 .7 0.3*1.1 0.4*1.1],'Color','w',...
                        'name',strcat('Simulation at 0'));
                    agent.cell_story(agent.plotindex) = agent.cell; 
                    agent.estimateStory(agent.plotindex) = agent.estimate;
                    agent.plotHandle = plot(1:agent.lenStory,agent.cell_story); hold on
                    agent.plotHandle2 = plot(1,agent.cell,'*'); axis([0 agent.lenStory 0 arena.cellNumber+1])
                    agent.plotHandle3 = plot(1:agent.lenStory,agent.estimateStory,'--');
                end
                
            else
                disp('Agent defined with default properties, ok?')
                pause
            end
        end
        
        function dlt = get.dlt(agent)          
            dlt = agent.rcoll^2*pi;
        end    
        
        function distanceTollerance = get.distanceTollerance(agent)          
            distanceTollerance = agent.rcoll .* 0.9;
        end  
        
    end
    
    methods
                
        function [v,w] = controller(agent,data,new_goal) 
            % update robot information
            agent.time = agent.time + agent.dt;                                                 % update global timer
            agent.measureTime = agent.measureTime + agent.dt;                                   % update collision filter timer
            agent.myState = data;                                                               % update robot state
            dist2goal = sqrt( (agent.myState(1)-agent.goal(1,1)).^2 + (agent.myState(2)-agent.goal(2,1)).^2 );
            v = agent.speed;                                                                    % initialize v to cruise speed
            
            % ==== Find Robot Cell ====
            agent.cell = findCell(agent);
            %if agent.transitioning == 1; agent.cell = findCell(agent,agent.goal(:,2)); end
           
            % ==== Draw a new cell and Assign Goal ======
            if mod(round(agent.time,2),agent.cellDwell*agent.dt) == 0 && ~agent.transitioning
                newcell = find( cumsum(agent.M(:,agent.cell)) > random('unif',0,1), 1 , 'first' );  % compute new cell
                if newcell ~= agent.cell
                    agent.goal = zeros(2,2);
                    agent.transitioning = 1;
                    agent.goal(:,1) = randOnInterface(agent,agent.cell);
                    agent.goal(:,2) = randInPoly(agent.Grid{newcell});
                end
            end  

            if dist2goal <= agent.distanceTollerance;
               if agent.transitioning; agent.transitioning = 0; end
               agent.goal(:,1) = agent.goal(:,2); 
               agent.goal(:,2) = randInPoly(agent.Grid{findCell(agent,agent.goal(:,2))}); % assign upcoming goal in the same cell as the current one
            end
            
            % ======== Check if in Collision ==============
            if agent.loms == 1 
%                 if ~inpolygon(new_goal(1,1),new_goal(2,1),agent.Grid{agent.cell}(1,:),agent.Grid{agent.cell}(2,:));     % assign new goal if collision resolving goal is not inside the current cell
%                     agent.waitingTime = 40;
%                     new_goal = randInPoly(agent.Grid{agent.cell});
%                 end

                if abs(agent.err2goal) > agent.headindsTollerance && dist2goal > agent.distanceTollerance
                    v = 0;                                          % if in collision and not on right heading yet set speed to 0
                end
                agent.goal(:,1) = new_goal;
                agent.goal(:,2) = randInPoly(agent.Grid{agent.cell});   
            end
                       
            
            % ============ Compute Controls =========
            w = PID(agent);
            if agent.waitingTime > 0; v = 0; agent.waitingTime = agent.waitingTime - 1; end     % if in wait mode set v = 0
            
            % ============ Run Collision Filter =============
            if mod(round(agent.measureTime,2),agent.measureWindow*agent.dt) == 0
                % ------------- Bayesian filter -------------
                %agent.estimate = BayesFilter(agent);
                % --------------------------------------
                % ----------- Viterbi algorithm ---------------
                agent.estimate = VITfilter(agent);
                % --------------------------------------
                agent.measureTime = 0;
                agent.nCollisions = 0;
              
                % update estimation plot
                if agent.selected == true; 
                    agent.plotindex = agent.plotindex + 1;
                    if mod(agent.plotindex,agent.lenStory) == 0
                        agent.plotindex = 1;
                    end
                    agent.cell_story(agent.plotindex) = agent.cell;
                    agent.estimateStory(agent.plotindex) = agent.estimate;
                    set(agent.plotHandle,'ydata',agent.cell_story);  
                    set(agent.plotHandle2,'xdata',agent.plotindex,'ydata',agent.cell);
                    set(agent.plotHandle3,'ydata',agent.estimateStory);      
                end
            end
        end
        
        function agent = addCollisions(agent,collisions)
            agent.nCollisions = agent.nCollisions + collisions;
        end
        
        function agent = resetWaitTimer(agent)
            agent.waitingTime = 0;
        end
    end
        
    methods(Access = 'private')   
        function w = PID(agent)
            persistent E;
            if isempty(E), E = 0; end
            G = atan2(agent.goal(2,1) - agent.myState(2),agent.goal(1,1) - agent.myState(1));   
            
            if rem(agent.time,100) == 0; E = 0; end

            pphi = agent.myState(3);
            G = rad2deg(G);

            d180 = (wrapTo180(G) - wrapTo180(rad2deg(pphi)));
            d360 = (wrapTo360(G) - wrapTo360(rad2deg(pphi)));
            derr = [d180,d360];
            [~,which] = min(abs(derr));
            error = deg2rad(derr(which));
            agent.err2goal = error;
            E = E + error;
            % ============ PI Controller ===============
            kp = 10*agent.dt;
            ki = 0.0005*agent.dt;
            w = (kp*error + ki*E) / agent.dt;        
            if abs(w) > agent.wMax; w = agent.wMax*w/abs(w); end % max ang.vel. limit 
        end
        
        function agentisincell = findCell(agent,point)
            % recursevely try each cell until find the correct one (ci = 1); if point is empty return cell where agent is at, ow return cell point belongs to
            if nargin == 1; point = agent.myState(1:2,1); end
            ci = 0;
            agentisincell = 0;
            while ~ci 
                agentisincell = agentisincell + 1;
                if agentisincell > size(agent.grid,2); 
                    error('No cell found!'); 
                end          
                ci = inpolygon(point(1,1),point(2,1),agent.grid{agentisincell}(1,:),agent.grid{agentisincell}(2,:));         
            end                
        end
        
        function estimate = BayesFilter(agent)
            
            lambda = agent.measureWindow*agent.dlt.*(agent.nAgents*agent.rho)./agent.Dlt;
            lambda = 0 + lambda.*(lambda>0);
            
            L = poisspdf(agent.nCollisions,lambda);
            Pr = L.*(agent.belief);
            agent.belief = (Pr)/sum(sum(Pr));
            [~, estimate] = max(agent.belief);        % mode critieria
            
            if agent.selected == true 
                disp('-------------')
                disp(['n of coll  :',num2str(agent.nCollisions)]);
                disp(['lambda     :',num2str(lambda')]);
                disp(['likelihood :',num2str(L')]);
                disp(['post       :',num2str(agent.belief')]);
                disp(['MAP        :',num2str(estimate)]);
                disp('.           .')
            end
        end
        
        function estimate = VITfilter(agent)
            
            agent.mu = agent.M * agent.mu;                            % update expected swarm distribution
            agent.phi = (agent.nAgents.*agent.dlt).*inv(diag(agent.Dlt))*agent.mu;
            G = agent.phi';
            if agent.nCollisions == 0                                    % choose observation matrix row
                G = 1 - G;
            end
            
            bel_new = zeros(size(agent.grid,2),1);                           % probability of cells occupancy
            for ii = 1:size(agent.grid,2)
                mx = 0;
                for jj = 1:size(agent.grid,2)
                    mx = max(mx,agent.M(ii,jj)*agent.belief(jj));
                end
                bel_new(ii) = G(ii) * mx; 
            end
            
            agent.belief = bel_new;                                 % agent belief
            [~,estimate] = max(agent.belief);                           % argmax (belief)
            
            if agent.selected == true 
                disp('-------------')
                disp(['\mu        :',num2str(agent.mu')]);
                disp(['n of coll  :',num2str(agent.nCollisions)]);
                disp(['G          :',num2str(G(1,:))]);
                disp(['belief     :',num2str(agent.belief')]);
                disp('.           .')   
            end
            
        end
        
    end
    
end