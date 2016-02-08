% Collision filtering - Dynamic case -> Estimate position on number of collisions measure

function robotArena(arena,r,khepera)
%  ===================== STOP BUTTON =====================
S.fh = figure('units','pix','pos',[200 100 170 130],'menubar','none','numbertitle','off','resize','off');
S.pb = uicontrol('string','Stop Simulation!','callback',{@pb_call},'units','pixels',...
                 'fontsize',15,'Backgroundcolor','r','fontweight','bold','position',[10 10 150 110]);
drawnow; pause(0.01) % ===================================


% Automatically select case
if nargin == 3                                                  % matlab simulation 
    data = OSupdate(r,khepera);                                 % pull new data
    type = 'sim';
elseif nargin == 2                                              % robotarium
    data = r.getPoses();                                        % pull new data
    type = 'robo';
    % ========= Log Data File Init ===============              % File for registration of robot data
    logid = fopen(['datalog',char(datetime),'.txt'],'w');       % open file for simulation data recording
    fprintf(logid,'%s\r\n',' *** New Robotarium Session ***');
else
    error('Unexpected number of input variables');
end

% =========== Create Agent Objects ==============
for aa = 1:arena.N              
    agent(aa) = Agent(aa,arena,data(:,aa));
end

screenPlot = ArenaPlot('screen',arena,agent);                   % create domain plot

switch type
    case 'sim'
        while ishandle(S.fh) 
            data = OSupdate(r,khepera);                                         % pull new robot data
            %[ arena.rho , cellOccupiers ] = findDistribution( [khepera.myState],arena );        % Agent distribution: fraction of agents in each cell
            [collCoords,new_goal] = evaluateCollisionsBETA(arena,data(1:2,:),agent);% evaluate collision coordinates and new computed goals
            %[collCoords,new_goal] = evaluateCollisions_beta(arena,data(1:2,:),agent,cellOccupiers);% evaluate collision coordinates and new computed goals

            
            for aa = 1:arena.N
                [V,W] = controller(agent(aa),data(:,aa),new_goal(:,aa));        % compute agent controls
                updateDynamics(khepera(aa),V,W);                                % send control command to robots
            end
            updatePlot(screenPlot,arena,agent,collCoords);                      % update plot
            pause(0.01); drawnow;
        end
        %for aa = 1:arena.N; move(khepera(aa),0,0); end                          % stop robots before return
   
    case 'robo'
        while ishandle(S.fh) 
            data = r.getPoses();                                                % pull new robot data
            [collCoords,new_goal] = evaluateCollisions(arena,data(1:2,:),agent);% evaluate collision coordinates and new computed goals
            V = zeros(1,arena.N); W = zeros(1,arena.N);                         % initialize empty control vectors
            for aa = 1:arena.N                        
                [v,w] = controller(agent(aa),data(:,aa),new_goal(:,aa));        % compute agent controls
                V(aa) = v; W(aa) = w;                                           % concatenate control vectors
            end
            r.setVelocities([V; W]);                                            % set computed controls                        
            r.updateDynamics();                                                 % update robot dynamics
            updatePlot(screenPlot,arena,agent,collCoords);
            drawnow;
            
            % ---------------------------- Print robot data ----------------------------
            fprintf(logid,'Agent ID - position - goal coordinates - V W\n');
            for nag = 1:arena.N
               fprintf(logid,'%d:  %6.4f %6.4f %6.4f; %6.4f  %6.4f; %6.4f %6.4f\n',...
                   [nag data(1,nag) data(2,nag) data(3,nag) agent(nag).goal(1,1) agent(nag).goal(2,1) V(nag) W(nag)]);
            end
            % ----------------------------
      
        end      
        r.stopAllRobots;        % stop robots before return to main
        fclose(logid);
end

closeVideo(screenPlot);

function [] = pb_call(varargin)
    % Callback for pushbutton
    delete(S.fh)  % Delete the figure.
end

end


function [collCoords,new_goal] = evaluateCollisionsBETA(arena,data,agent)
    
persistent old_clusters; if isempty(old_clusters); old_clusters{1} = 0; end
N = arena.N;
collisionInAgents = zeros(1,N);             % initialize collision vector --> i-th element = 1 if i-th agent experiances collisions, 0 o/w
new_goal = [agent.goal];                    % initialize new goals as old goals  
[~,~,clusters] = collisionFinder( data , arena.ggp );
for cluster_ID = 1:numel(clusters)
    clu = clusters{cluster_ID};
    if ~matchCells(clu,old_clusters)        % if clu did not already existed 
        xcg = sum(data(1,clu))./size(clu,2);                              % center of collision x-coord 
        ycg = sum(data(2,clu))./size(clu,2);                              % center of collision y-coord

        for aa = clu %(REMOVE THIS FOR CYCLE!!!)
            agent(aa).loms = 1;                 % set agent collision flag to true
            addCollisions(agent(aa),1);         % add a collision to the number of collisions agent has registered
            new_heading = atan2( (agent(aa).myState(2)-ycg) , (agent(aa).myState(1)-xcg) );
            new_goal(:,aa) = [agent(aa).myState(1) + 1.1*arena.ggp*cos(new_heading);
                              agent(aa).myState(2) + 1.1*arena.ggp*sin(new_heading)];        
            collisionInAgents(aa) = 1;
        end
    end
end

agentInCollision = sort([clusters{:}]);
agentNotInCollision = setdiff(1:N,agentInCollision);
for agnot = agentNotInCollision
    agent(agnot).loms = 0;
end

old_clusters = clusters;
collCoords = [data(1,agentInCollision) ; data(2,agentInCollision)];   % list of positions of colliding agent (used for red marks in the plot)
%pause  
end

    
    
    function [collCoords,new_goal] = evaluateCollisions(arena,data,agent)
    % this function creates the list of agents experiencing a collision. Collisions are resolved creating a list of new
    % checkpoints (goal). Checkpoints are unchanged for those agents not experiencing a collision

    N = arena.N;
    collisionInAgents = zeros(1,N);             % initialize collision vector --> i-th element = 1 if i-th agent experiances collisions, 0 o/w
    new_goal = [agent.goal];                    % initialize new goals as old goals  
    [colliders,collided] = collisionFinder( data , arena.ggp );
    if ~isempty(colliders)
        setColl_unord = [colliders,collided];               
        [x,ilx] = unique(setColl_unord(:,1));                   % sort and consider only one collision for each agent
        setColl = [x,setColl_unord(ilx,2)];                     % ordered set of "colliders-collided"
        %setColl = colliders;
        %goal = [agent(x).goal];                                 % initialize vector for new goal to assign to colliding agents
        for idx = 1:size(setColl,1)                             % index running from 1 to # of agents in collision
            if agent(setColl(idx,1)).loms == 0                  % if agent where not in a collision already
                agent(setColl(idx,1)).loms = 1;                 % set agent collision flag to true
                addCollisions(agent(setColl(idx,1)),1);         % add a collision to the number of collisions agent has registered
                % New goal located at -1.1 agent radius along current heading (backward)
                rdn = 0.05 -0.1.*rand(1,1);
                new_goal(:,setColl(idx,1)) = [agent(setColl(idx,1)).myState(1) - 1.1*arena.ggp*cos(agent(setColl(idx,1)).myState(3) + rdn);
                                              agent(setColl(idx,1)).myState(2) - 1.1*arena.ggp*sin(agent(setColl(idx,1)).myState(3) + rdn)];       
   
                
            end
            collisionInAgents(setColl(idx,1)) = 1;
        end
        
        notCollAgents = setdiff(1:N,x);                         % list of not colliding agents
        if ~isempty(notCollAgents)                              % clear collision flag for non-colliding agents
            for gg = notCollAgents; agent(gg).loms = 0; end
        end

        %new_goal(:,setColl(:,1)) = goal;                        % list of new gaol for agents listed in setColl, keep remaining unchanged
        %collisionInAgents(x) = 1;                               % index list of agent closer than tollerance
        
    else
        for nn = 1:N; agent(nn).loms = 0; end
    end

    collCoords = [data(1,collisionInAgents>0) ; data(2,collisionInAgents>0)];   % list of positions of colliding agent (used for red marks in the plot)
    
end







