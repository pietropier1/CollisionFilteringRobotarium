classdef Agent < handle
    
    % Class for agent    
    properties (SetAccess = private, GetAccess = public)
        ID = 1;                             % agent ID
        cell = 1;                           % cell agent is at
        selected;                           % is agent selected for tracking? (boolean)
        myState = [0;0;0];                  % agent state = [x;y;th] -> global reference
        grid = {[-1,1,1,-1;1,1,-1,-1]};     % arena grid info [center, corners in cw order from top left]
        Grid;                               % reduced grid for account robot dimension
        nAgents;                            % total number of agens
        Dlt;                                % Single cells area
        rcoll;                              % radius of robot area 
        DD = [];                            % N*dlt/Dlt ratio for each cell
        M;                                  % transition probability matrix
        err2goal;                           % error angle to goal
        rho;                                % swarm real distribution
        estimate;                           % estimated cell for robot location
        belief;                             % agent position belief
        dt;                                 % simulation time step size
        measureTime = 0;                    % time counter    
        time = 0;                           % simulation running time
        cell_story = [];                    % cell movements progression
        estimateStory = [];                 % estimate progression         
    end
    
    properties (Access = private)
        flagSlow = 0;                       % delay agent respone when resolving collision
        plotindex;                          % index for continuous plot
        plotHandle;                         % plot handle for agent actual position
        plotHandle2;                        % plot handle for marker
        plotHandle3;                        % plot handle for belief
    end
        
    
    properties (SetAccess = public)
        goal;                               % agent coverage task checkpoint [xg;yg]
        loms = 0;                           % loss of minimum separation flag
        nCollisions = 0;                    % number of collision
    end
    
    properties(Constant)
        wMax = 2;                           % max angular velocity
        lenStory = 1000;                    % length of continuous plot
        measureWindow = 50;                 % collision observation window time length
        speed = 0.06;                       % linear velocity
    end
    
    properties(Dependent)
        dlt;
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
                agent.nAgents = arena.N;
                agent.myState = data;   
                agent.cell = findCell(agent);
                agent.Dlt = arena.Dlt;
                agent.M = arena.M;
                agent.dt = arena.dt;
               
                agent.rho = arena.rho./(sum(arena.rho));
                agent.DD = (agent.nAgents.*agent.dlt)./agent.Dlt;
                agent.belief = agent.rho; 
                [~, agent.estimate] = max(agent.belief);
                % ====== Assign initial random goal inside current cell ===
                agent.goal = randInPoly(agent.Grid{agent.cell});
                
                % ======== Create estimation plot =============
                if agent.selected == true; agent.plotindex = 1; 
                    FH = figure();
                    set(FH,'units','normalized','position',[.5 .7 0.3*1.1 0.4*1.1],'Color','w',...
                        'name',strcat('Simulation at 0'));
                    agent.cell_story(agent.plotindex) = agent.cell; 
                    agent.estimateStory(agent.plotindex) = agent.estimate;
                    agent.plotHandle = plot(1:agent.lenStory,agent.cell_story); hold on
                    agent.plotHandle2 = plot(1,agent.cell,'*'); axis([0 agent.lenStory 0 4])
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
    end
    
    methods
                
        function [v,w] = controller(agent,data,new_goal) 
            % update robot information
            agent.time = agent.time + agent.dt;
            agent.myState = data;
            agent.measureTime = agent.measureTime + agent.dt; 
           
            % compute new cell for agent accordingly to M matrix probability distribution
            newcell = find( cumsum(agent.M(:,agent.cell)) > random('unif',0,1), 1 , 'first' );
            
            dist2goal = sqrt( (agent.myState(1)-agent.goal(1)).^2 + (agent.myState(2)-agent.goal(2)).^2 );
            v = agent.speed;   
            
            % ======== Check on new cell =========
            if newcell == agent.cell
                if dist2goal < agent.rcoll; 
                    agent.goal = randInPoly(agent.Grid{newcell});
                end      
            else
                agent.goal = randInPoly(agent.Grid{newcell});
                agent.cell = newcell;           % agent considered in new cell even if not there yet
            end 

            if agent.flagSlow > 0; agent.flagSlow = agent.flagSlow - 1; end
            % ====== Check collision flag =========
            if agent.loms == 1 
                % if in collision and not on right heading yet do not move
                if abs(agent.err2goal) > 0.4 
                    v = 0;
                end
                
                % if the new goal from collision is not in current cell assign an alternative one but go there slowly
                if ~inpolygon(new_goal(1,1),new_goal(2,1),agent.grid{agent.cell}(1,:),agent.grid{agent.cell}(2,:)); 
                    new_goal = randInPoly(agent.Grid{agent.cell});
                    agent.flagSlow = 20;                % when alternative goal is assigned a lower gain is assigned, allowing for the collided neighbor to move first.
                end
                agent.goal = new_goal;
                agent.cell = findCell(agent,agent.goal);
            end  
            w = PID(agent);
            if abs(w) > agent.wMax; w = agent.wMax*w/abs(w); end % max ang.vel. limit 
            
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
            end  
            
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
        
        function agent = addCollisions(agent,collisions)
            agent.nCollisions = agent.nCollisions + collisions;
        end
        
    end
        
    methods(Access = 'private')   
        function w = PID(agent)
            persistent E;
            if isempty(E), E = 0; end
            G = atan2(agent.goal(2) - agent.myState(2),agent.goal(1) - agent.myState(1));   
            
            if rem(agent.time,100) == 0; E = 0; end

            phi = agent.myState(3);
            G = rad2deg(G);

            d180 = (wrapTo180(G) - wrapTo180(rad2deg(phi)));
            d360 = (wrapTo360(G) - wrapTo360(rad2deg(phi)));
            derr = [d180,d360];
            [~,which] = min(abs(derr));
            error = deg2rad(derr(which));
            agent.err2goal = error;
            E = E + error;
            % PI Controller
            kp = 3*agent.dt;
            if agent.flagSlow > 0; kp = kp/10; end % reduce p gain when alternative goal has been assigned after collision
            ki = 0.005*agent.dt;
            w = (kp*error + ki*E) / agent.dt;         
        end
        
        function agentisincell = findCell(agent,point)
            % recursevely try each cell until find the correct one (ci = 1); if point is empty return cell where agent is at, ow return cell point belongs to
            if nargin == 1; point = agent.myState(1:2,1); end
            ci = 0;
            agentisincell = 0;
            % disp(['Agent',num2str(agent.ID),'is in ',num2str(agent.myState')])
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
            agent.rho = agent.M * agent.rho;                            % update expected swarm distribution
            if agent.nCollisions > 0                                    % choose observation matrix row
                G = agent.rho'.*agent.DD;
            else
                G = ones(1,size(agent.rho,1)) - agent.rho'.*agent.DD;
            end
            
            mu = zeros(size(agent.grid,2),1);                           % probability of cells occupancy
            for ll = 1:size(agent.grid,2)
                mu(ll) = G(1,ll) .* max(agent.M(ll,:) .* agent.belief'); 
            end
            
            agent.belief = mu./sum(mu);                                 % agent belief
            [~,estimate] = max(agent.belief);                           % argmax (belief)
            
            if agent.selected == true 
                disp('-------------')
                disp(['\rho       :',num2str(agent.rho')]);
                disp(['n of coll  :',num2str(agent.nCollisions)]);
                disp(['G          :',num2str(G(1,:))]);
                disp(['belief     :',num2str(agent.belief')]);
                disp('.           .')   
            end
            
        end
        
    end
    
end