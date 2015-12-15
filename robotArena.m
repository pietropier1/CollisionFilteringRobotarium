% Collision filtering - Dynamic case -> Estimate position on number of collisions measure

function robotArena(arena,r,khepera)
%  ===================== STOP BUTTON =====================
S.fh = figure('units','pix','pos',[200 100 170 130],'menubar','none','numbertitle','off','resize','off');
S.pb = uicontrol('string','Stop Simulation!','callback',{@pb_call},'units','pixels',...
                 'fontsize',15,'Backgroundcolor','r','fontweight','bold','position',[10 10 150 110]);
drawnow; pause(0.01) % ===================================

% ========= Log Data File Init ===============
logid = fopen(['datalog',char(datetime),'.txt'],'w');
fprintf(logid,'%s\r\n',' *** New Robot Session ***');
% ============================================


% Select simulation case 
if nargin == 3                  % matlab simulation 
    data = OSupdate(r,khepera);
    type = 'sim';
elseif nargin == 2              % robotarium
    data = r.getPoses();
    type = 'robo';
else
    error('Unexpected number of input variables');
end

% =========== Create Agent Objects ==============
for aa = 1:arena.N              % create agents
    agent(aa) = Agent(aa,arena,data(:,aa));
end
% ===============================================

screenPlot = ArenaPlot('screen',arena,agent);       % plots

switch type
    case 'sim'
        while ishandle(S.fh) 
            data = OSupdate(r,khepera);
            [collCoords,new_goal] = evaluateCollisions(arena,data(1:2,:),agent);

            for aa = 1:arena.N                        % update cell occupancy
                [V,W] = motion(agent(aa),data(:,aa),new_goal(:,aa));
                move(khepera(aa),V,W);
            end

            updatePlot(screenPlot,arena,agent,collCoords);
            pause(0.01); drawnow;
        end
        for aa = 1:arena.N; move(khepera(aa),0,0); end % stop robots
   
    case 'robo'
        while ishandle(S.fh) 
            data = r.getPoses();
            [collCoords,new_goal] = evaluateCollisions(arena,data(1:2,:),agent);
            V = zeros(1,arena.N); W = zeros(1,arena.N);
            for aa = 1:arena.N                        % update cell occupancy
                [v,w] = motion(agent(aa),data(:,aa),new_goal(:,aa));
                V(aa) = v; W(aa) = w;
            end
%             disp(num2str(V))
%             disp(num2str(W))
%             disp('****')
            r.setVelocities([V; W]);
          
            drawnow;
            
            % ---------------------------- Print robot data ----------------------------
            fprintf(logid,'Agent ID - position - goal coordinates - V W\n');
            for nag = 1:arena.N
               fprintf(logid,'%d:  %6.4f %6.4f %6.4f; %6.4f  %6.4f; %6.4f %6.4f\n',...
                   [nag data(1,nag) data(2,nag) data(3,nag) agent(nag).goal(1,1) agent(nag).goal(2,1) V(nag) W(nag)]);
            end
            % ----------------------------
            
            r.updateDynamics();
            updatePlot(screenPlot,arena,agent,collCoords);
            
        end      
        r.stopAllRobots;        % stop robots
        
end

fclose(logid);
closeVideo(screenPlot);

function [] = pb_call(varargin)
    % Callback for pushbutton
    delete(S.fh)  % Delete the figure.
end

end


function [collCoords,new_goal] = evaluateCollisions(arena,data,agent)

    N = arena.N;
    collisionInAgents = zeros(1,N);           % i-th element = 1 if i-th agent experiances collisions, 0 o/w

    new_goal = [agent.goal];
    DD = pdist2(data',data');
    DD = DD + diag(ones(1,N));             % filter diagonal and repeated elements
    [colliders,collided] = ind2sub([N,N],find(DD < 2*agent(1).rcoll));
    if ~isempty(colliders)
        setColl_unord = [colliders,collided];
        [x,ilx] = unique(setColl_unord(:,1));                % sort and consider only one collision for each agent
        setColl = [x,setColl_unord(ilx,2)];                  % ordered set of "colliders-collided"
        goal = [agent(x).goal]; % zeros(2,size(setColl,1));
        for idx = 1:size(setColl,1)
            if agent(setColl(idx,1)).loms == 0
                agent(setColl(idx,1)).loms = 1;
                addCollisions(agent(setColl(idx,1)),1);
                goal(:,idx) = [agent(setColl(idx,1)).myState(1) - 0.1*cos(agent(setColl(idx,1)).myState(3));
                               agent(setColl(idx,1)).myState(2) - 0.1*sin(agent(setColl(idx,1)).myState(3))];%agent((setColl(idx,2))).goal;
            end
        end
        notCollAgents = setdiff(1:N,x);
        if ~isempty(notCollAgents)
            for gg = notCollAgents; agent(gg).loms = 0; end
        end
        
        new_goal(:,setColl(:,1)) = goal;

        collisionInAgents(x) = 1;
    else
        for nn = 1:N; agent(nn).loms = 0; end
    end

    collCoords = [data(1,collisionInAgents>0) ; data(2,collisionInAgents>0)];

end







