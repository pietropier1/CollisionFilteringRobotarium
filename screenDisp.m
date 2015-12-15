function screenDisp(arena,agent,tt)


disp(strcat('========= Iteration number:',num2str(tt),' ==========')); disp('');
for jj = 1:arena.cellNumber
    disp('.........................');
    disp(strcat( 'Cell ',num2str(jj) ));
    disp('.........................');
    for ii = arena.cellOccup{jj}
       disp(strcat( 'Agent ',num2str(ii),' - number of collision:',num2str(agent(ii).nCollisions) )); 
    end
end

disp('****************************');
disp(' ');

[agent.state]
%[agent.nCollisions]
end