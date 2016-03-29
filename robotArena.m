% This function runs the actual simulation. Simulation ('sim' case) or experiment ('robo' case) are automatically detected based
% on number of inputs. Both 'sim' and 'robo' performes the same operations and call the same functions. In order:
% 1. update robots' poses (in 'sim' case virtual agents called khepera are used)
% 2. evaluate if some collision is occuring and compute new goals for their resolution
% 3. compute controls for each agent and set controls to agents
% 4. update plot



function agent = robotArena(arena,r,khepera)
%  ===================== STOP BUTTON =====================
in_SS = get(0,'screensize');
S.fh = figure('units','pix','pos',[0.85*in_SS(1,3) 0.8*in_SS(1,4) 170 130],'menubar','none','numbertitle','off','resize','off');
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
            [collCoords,new_goal] = evaluateCollisions(arena,data(1:2,:),agent);% evaluate collision coordinates and new computed goals
     
            for aa = 1:arena.N
                [V,W] = controller(agent(aa),data(:,aa),new_goal(:,aa));        % compute agent controls
                updateDynamics(khepera(aa),V,W);                                % send control command to robots
            end
            updatePlot(screenPlot,arena,agent,collCoords);                      % update plot
            pause(0.01); drawnow;
        end
   
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



function [collCoords,goal] = evaluateCollisions(arena,data,agent)
% Evaluate clusters of agents in collision and compute new goals 
% collCoords --> coordinates of agents in collision (used only for plot purposes)
% new_goal --> new goals for each agent (the goals for agents not experiencing a new collision are unchanged)

persistent old_clusters; if isempty(old_clusters); old_clusters{1} = 0; end
N = arena.N;
goal = [agent.goal];                                    % initialize new goals as old goals  
goal = goal(:,1:2:end);                                 % consider only the first goal
[~,~,clusters] = collisionFinder( data , arena.ggp );   % clusters of agents in collision
for cluster_ID = 1:numel(clusters)                      % for each cluster
    clu = clusters{cluster_ID};                         % agents of this cluster
    if ~matchCells(clu,old_clusters)                    % if clu did not already existed in old_clusters 
        xcg = sum(data(1,clu))./size(clu,2);            % center of mass of collision (x-coord) 
        ycg = sum(data(2,clu))./size(clu,2);            % center of mass of collision (y-coord)

        for aa = clu                                    % for each agent in the cluster
            agent(aa).loms = 1;                         % set agent collision flag to true
            addCollisions(agent(aa),1);                 % add a collision to the number of collisions agent has registered
        end
        new_heading = atan2( (data(2,clu)-ycg) , (data(1,clu)-xcg) );       % direction to cluster center of mass
        goal(:,clu) = [data(1,clu) + 1.3*arena.ggp.*cos(new_heading);       % new goal away from center of mass
                       data(2,clu) + 1.3*arena.ggp.*sin(new_heading)];   
    end
    
    for ag = clu
        if agent(ag).waitingTime > 0
            
            nclu = setdiff(clu,ag);                     % set difference clu \ ag
            conf = conflictHeadings(data(:,ag),goal(:,ag),data(:,nclu),goal(:,nclu),arena.ggp);
            if conf; 
                goal(:,ag) = [1e3;1e3]; % if there is conflict assign dummy goal outside cell
            end                                                        
        end
    end      
end

agentInCollision = sort([clusters{:}]);
if size(agentInCollision,2) > arena.N; agentInCollision = 1:arena.N; end
agentNotInCollision = setdiff(1:N,agentInCollision);
for agnot = agentNotInCollision
    agent(agnot).loms = 0;
end

old_clusters = clusters;
collCoords = [data(1,agentInCollision) ; data(2,agentInCollision)];   % list of positions of colliding agent (used for red marks in the plot)
end




% DEVELOPING !!!

function [collCoords,goal] = evaluateCollisionsBETA(arena,data,agent)
% Evaluate clusters of agents in collision and compute new goals 
% collCoords --> coordinates of agents in collision (used only for plot purposes)
% new_goal --> new goals for each agent (the goals for agents not experiencing a new collision are unchanged)

persistent old_clusters; if isempty(old_clusters); old_clusters{1} = 0; end
N = arena.N;
goal = [agent.goal];                                    % initialize new goals as old goals  
goal = goal(:,1:2:end);                                 % consider only the first goal
[~,~,clusters] = collisionFinder( data , arena.ggp );   % clusters of agents in collision
collCoords = zeros(2,N);
for cluster_ID = 1:numel(clusters)                      % for each cluster
    clu = clusters{cluster_ID};                         % agents of this cluster
    if ~matchCells(clu,old_clusters)                    % if clu did not already existed in old_clusters 
        xcg = sum(data(1,clu))./size(clu,2);            % center of mass of collision (x-coord) 
        ycg = sum(data(2,clu))./size(clu,2);            % center of mass of collision (y-coord)

        for aa = clu                                    % for each agent in the cluster
            agent(aa).loms = 1;                         % set agent collision flag to true
            addCollisions(agent(aa),1);                 % add a collision to the number of collisions agent has registered
        end
%         new_heading = atan2( (data(2,clu)-ycg) , (data(1,clu)-xcg) );       % direction to cluster center of mass
%         goal(:,clu) = [data(1,clu) + 1.2*arena.ggp.*cos(new_heading);       % new goal away from center of mass
%                        data(2,clu) + 1.2*arena.ggp.*sin(new_heading)];  
        goal(:,clu) = deconOptimal(data(1:2,clu),goal(1:2,clu));           %reassign all cluster goals optimally
        conf = conflictHeadings(data(:,ag),goal(:,ag),data(:,nclu),goal(:,nclu),arena.ggp);         % check conflict with current goal assignment
        
        try           
        % check if new goal is unfeasible
        for aa = clu
            if ~inpolygon(goal(1,aa),goal(2,aa),arena.Grid{agent(aa).cell}(1,[1:end 1]) ,arena.Grid{agent(aa).cell}(2,[1:end 1]));     % if new goal is not inside the current cell
                 [xg , yg] = polyxpoly( [xcg,goal(1,aa)] , [ycg,goal(2,aa)] , arena.Grid{agent(aa).cell}(1,[1:end 1]) , arena.Grid{agent(aa).cell}(2,[1:end 1]));   
                 if isempty(xg); xg = xcg; yg = ycg; end
                 goal(:,aa) = [xg(1);yg(1)];
                 agent(aa).waitingTime = 5*randi(4,[1,1]);
            end
            
        end
        
        catch err
            display(err)
            display(['x1 =[', num2str([xcg,goal(1,aa)]),']'])
            display(['y1 =[', num2str([ycg,goal(2,aa)]),']'])
            display(['x2 =[', num2str(arena.Grid{agent(aa).cell}(1,:)),']'])
            display(['y2 =[', num2str(arena.Grid{agent(aa).cell}(2,:)),']'])
            inpolygon(goal(1,aa),goal(2,aa),arena.Grid{agent(aa).cell}(1,:),arena.Grid{agent(aa).cell}(2,:))
            [a,b] = polyxpoly( [data(1,aa);goal(1,aa)] , [data(2,aa);goal(2,aa)] , arena.Grid{agent(aa).cell}(1,:) , arena.Grid{agent(aa).cell}(2,:))
            pause
            return
        end
                   
    end
    
%     flag_rtl = 1;                                                               % flag run this loop (used to jup out of this loop if optimal assignment is performed)
%     for ag = clu                                                                % take on agent of the cluster
%         if agent(ag).waitingTime > 0  & flag_rtl                                % if this agent is on a wait, check if its heading is conflicting with some ngb
%             nclu = setdiff(clu,ag);                                             % set difference clu \ ag
%             conf = conflictHeadings(data(:,ag),goal(:,ag),data(:,nclu),goal(:,nclu),arena.ggp);         % check conflict with current goal assignment
%             while conf;                                                            % if this agent heading is on conflict 
%                 goal(:,clu) = deconOptimal(data(1:2,clu),goal(1:2,clu));           %reassign all cluster goals optimally
%                 conf = conflictHeadings(data(:,ag),goal(:,ag),data(:,nclu),goal(:,nclu),arena.ggp);
%             end   
%             for iag = clu;resetWaitTimer(agent(iag)); end                                          % reset waiting time to 0
%             flag_rtl = 0;
%             
%         end
%     end  
    
end

agentInCollision = sort([clusters{:}]);
if size(agentInCollision,2) > arena.N; agentInCollision = 1:arena.N; end
agentNotInCollision = setdiff(1:N,agentInCollision);
for agnot = agentNotInCollision
    agent(agnot).loms = 0;
end

old_clusters = clusters;
collCoords = [data(1,agentInCollision) ; data(2,agentInCollision)];   % list of positions of colliding agent (used for red marks in the plot)
end

    
    
 