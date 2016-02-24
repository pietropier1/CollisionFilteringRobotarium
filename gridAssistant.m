function gridAssistant(l,h,arena)

figure, hold on
step = 0.05;
xx = -l/2 : step : l/2;
yy = -h/2 : step : h/2;

[x,y] = meshgrid(xx,yy);
plot(x,y,'.k')
axis([-l/2-step,l/2+step,-h/2-step,h/2+step])

for jj = 1:arena.cellNumber
        plot([arena.grid{jj}(1,:) arena.grid{jj}(1,1)],[arena.grid{jj}(2,:) arena.grid{jj}(2,1)],'--k')
end

for jj = 1:arena.cellNumber
        plot(arena.Interface{jj}(1,:),arena.Interface{jj}(2,:),'r','Linewidth',2)
end

axis equal