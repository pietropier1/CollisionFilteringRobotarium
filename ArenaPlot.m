classdef ArenaPlot < handle
    % figure object for arena plot
    
    properties
        figH;
        PAgshape;            % Agent positions plot
        PAgdir;
        PTr;            % Agent Trajectory plot
        PCl;            % Collision spots plot
        PHeL;
        PGl;
        PAgHero;
        MMvie = [];
        trajct;
        plotTailBuffer = 14;    % lenght of the tail path trace
        frameIndex = 0; 
        PslcCell;           % patch for highlight estimated cell
        hero;           % main character agent
    end
    
    properties (Constant)
        
    end
    
    methods % constructor
        function arenaPlot = ArenaPlot(position,arena,agent)
            arenaPlot.hero = arena.agSel;
            arenaPlot.figH = figure();
            switch position
                case 'screen'
                    set(arenaPlot.figH,'units','normalized','position',[.05 .5 0.3*1.1 0.4*1.1],'Color','w',...
                        'name',strcat('Simulation at 0'));
                    hold on; box on
                    gridComp = [arena.grid{:}];        % compress grid data
                    set(gca,'Position',[.05 .05 .9 .9],'Color',[0.5,0.5,0.5],...
                        'XLim',[min(min(gridComp))-0.2,max(max(gridComp)) + 0.2],...
                        'YLim',[min(min(gridComp))-0.2,max(max(gridComp)) + 0.2])           
                    xpatch = reshape(gridComp(1,:),size(arena.grid{1},2),size(arena.grid,2));
                    ypatch = reshape(gridComp(2,:),size(arena.grid{1},2),size(arena.grid,2));
                    p = patch(xpatch,ypatch,'w');
                    set(p,'FaceColor','w','EdgeColor','k','Linewidth',3);
                    
                    
                case 'projector'                
                    
            end
            arenaPlot.PslcCell =  patch(0.99*gridComp(1,1:size(arena.grid{1},2)),0.99*gridComp(2,1:size(arena.grid{1},2)),'w');
            set(arenaPlot.PslcCell,'FaceAlpha',0,'EdgeColor','g','Linewidth',3);
            arenaPlot.trajct = 2e5.*ones(arenaPlot.plotTailBuffer,2*arena.N); % agent dummy trajectories
            arenaPlot.PTr = plot(arenaPlot.trajct(:,1:arena.N) , arenaPlot.trajct(:,arena.N+1:end),'.','Color',[.1 .49 .63],'markersize',8);
            arenaPlot = createAgentsplot(arenaPlot,agent);
            
            if arena.makemovie
                arenaPlot.MMvie = VideoWriter(['CollisionTrack',char(datetime)]);%,'MPEG-4'); 
                set(arenaPlot.MMvie,'FrameRate',8); 
                open(arenaPlot.MMvie);
            end
        
        end
        % ----------------
        function arenaPlot = createAgentsplot(arenaPlot,agent)      
            N = agent(end).ID;
            robStates = [agent.myState]; 
            r = agent(1).rcoll;
            p = repmat(linspace(0,2*pi,25),N,1)';        % circle parameter
            xState = robStates(1,:);
            yState = robStates(2,:);
            tState = robStates(3,:);
            % create patch for collision marks
            arenaPlot.PCl = patch(repmat(1e5*ones(1,N),25,1) + 1.2.*r.*cos(p),...
                repmat(1e5*ones(1,N),25,1) + 1.2*r.*sin(p),'g'); hold on
            set(arenaPlot.PCl,'FaceColor','r','Facelighting','flat','EdgeColor','r','FaceAlpha',0.5);
            
            % create patch for all agent (hero included)
            arenaPlot.PAgshape = patch(repmat(xState,25,1) + r.*cos(p),...
                repmat(yState,25,1) + r.*sin(p),'g'); hold on
            set(arenaPlot.PAgshape,'FaceColor',[0.8,0.8,0.8],'Facelighting','flat','EdgeColor','k','LineWidth',2);
            
            % create patch for hero agent
            arenaPlot.PAgHero = patch(repmat(xState(arenaPlot.hero),1,25) + r.*cos(linspace(0,2*pi,25)),...
                repmat(yState(arenaPlot.hero),1,25) + r.*sin(linspace(0,2*pi,25)),'g'); hold on
            set(arenaPlot.PAgHero,'FaceColor',[0.3,0.8,0.3],'Facelighting','flat','EdgeColor','k','LineWidth',2);
            arenaPlot.PGl = plot(agent(arenaPlot.hero).goal(1),agent(arenaPlot.hero).goal(2),'g*');

            % create heading indicator
            arenaPlot.PHeL = plot([xState; xState+1.2*r.*cos(tState)] , [yState; yState+1.2*r.*sin(tState)],'k','linewidth',2);
        end
    end
    
    
    methods
        % ---------------
        function updatePlot(arenaPlot,arena,agent,collCoords)  
            
            set(arenaPlot.figH,'name',strcat('Simulation at',' ',num2str(agent(1).time)))
            if arenaPlot.frameIndex == arenaPlot.plotTailBuffer; 
                arenaPlot.frameIndex = 0; 
            end
            arenaPlot.frameIndex = arenaPlot.frameIndex+1;
            
            robStates = [agent.myState];
            %arenaPlot.trajct(arenaPlot.frameIndex,:) = [robStates(1,:), robStates(2,:)];  % build trajecotries for plotting            
            %set(arenaPlot.PTr,'xdata',reshape(arenaPlot.trajct(:,1:arena.N),[arenaPlot.plotTailBuffer*arena.N,1]),...
            %              'ydata',reshape(arenaPlot.trajct(:,arena.N+1:end),[arenaPlot.plotTailBuffer*arena.N,1]));
            
            arenaPlot = updateAgentsplot(arenaPlot,agent,collCoords);
            
            highLgt = agent(arenaPlot.hero).estimate;
            set(arenaPlot.PslcCell,'xdata',arena.grid{highLgt}(1,:),'ydata',arena.grid{highLgt}(2,:));
            if ~isempty(arenaPlot.MMvie);
                writeVideo( arenaPlot.MMvie, getframe(arenaPlot.figH) );
            end
        end
        
        % ----------------
        function arenaPlot = updateAgentsplot(arenaPlot,agent,collCoords)
            
            robStates = [agent.myState];
            N = agent(end).ID;
            r = agent(1).rcoll;
            p = repmat(linspace(0,2*pi,25),N,1)';           % circle parameter
            %update agent positions
            set(arenaPlot.PAgshape,'xdata',repmat(robStates(1,:),25,1) + r.*cos(p),'ydata',...
                repmat(robStates(2,:),25,1) + r.*sin(p));
            
            collxy = 1e5*ones(2,N);
            collxy(:,1:size(collCoords,2)) = collCoords;
            % update collision points
            set(arenaPlot.PCl,'xdata',repmat(collxy(1,:),25,1) + 1.5.*r.*cos(p),'ydata',...
                repmat(collxy(2,:),25,1) + 1.5.*r.*sin(p));
            % update hero position
            set(arenaPlot.PAgHero,'xdata',repmat(robStates(1,arenaPlot.hero),1,25) + r.*cos(linspace(0,2*pi,25)),'ydata',...
                repmat(robStates(2,arenaPlot.hero),1,25) + r.*sin(linspace(0,2*pi,25)));
            set(arenaPlot.PGl,'xdata',agent(arenaPlot.hero).goal(1),'ydata',agent(arenaPlot.hero).goal(2));
            % update heading indicators
            set(arenaPlot.PHeL,{'xdata'},num2cell([robStates(1,:); robStates(1,:)+1.2*r.*cos(robStates(3,:))]',2),...
                               {'ydata'},num2cell([robStates(2,:); robStates(2,:)+1.2*r.*sin(robStates(3,:))]',2));
        end
        
        function closeVideo(ArenaPlot)
            if ~isempty(ArenaPlot.MMvie)
                close(ArenaPlot.MMvie);
            end
        end
        
    end
    
end

