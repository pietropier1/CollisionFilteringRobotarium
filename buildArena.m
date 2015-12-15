function [ arena,InitConf ] = buildArena( type,arena )
% build arena features
switch type
    
    case '4blocks'
        arena.cellNumber = 4;             % x direction # of cell
        l = 1.2;
        h = 0.7;
        arena.grid{1} = [-l/2,0,0,-l/2;h/2,h/2,0,0];
        arena.grid{2} = [0,l/2,l/2,0;h/2,h/2,0,0];
        arena.grid{3} = [0,l/2,l/2,0;0,0,-h/2,-h/2];
        arena.grid{4} = [-l/2,0,0,-l/2;0,0,-h/2,-h/2];
        arena.Grid{1} = dilationPoly(arena.grid{1},arena.ggp);
        arena.Grid{2} = dilationPoly(arena.grid{2},arena.ggp);
        arena.Grid{3} = dilationPoly(arena.grid{3},arena.ggp);
        arena.Grid{4} = dilationPoly(arena.grid{4},arena.ggp);
        InitConf{1} = [1];
        InitConf{2} = [2 3];%[4 5 6 7 8];
        InitConf{3} = [4 5];%[9 10 11 12 13 14 15]; 
        InitConf{4} = [6];
        
    case '3tria'
        l = 0.6;
        h = 0.7;
        arena.cellNumber = 3;             % x direction # of cell
        arena.grid{1} = [-l,0,-l/2; h/2,h/2,-h/2];
        arena.grid{2} = [0,l/2,-l/2; h/2,-h/2,-h/2];
        arena.grid{3} = [0,l,l/2; h/2,h/2,-h/2];
        arena.Grid{1} = dilationPoly(arena.grid{1},arena.ggp);
        arena.Grid{2} = dilationPoly(arena.grid{2},arena.ggp);
        arena.Grid{3} = dilationPoly(arena.grid{3},arena.ggp);
        InitConf{1} = [1];
        InitConf{2} = [2 3];%[4 5 6 7 8];
        InitConf{3} = [4 5 6];%[9 10 11 12 13 14 15]; 
        switch 2
            case 1
                arena.M = [0.9660    0.0170    0.0000
                           0.0340    0.9434    0.0113
                           0.0000    0.0396    0.9887];
            case 2
                arena.M = eye(arena.cellNumber,arena.cellNumber);
            case 3
                arena.M = [0.9000    0.0200    0.0200
                           0.0400    0.9200    0.0400
                           0.0600    0.0600    0.9400];        
        end
        
    case '4tria'
        l = 1;
        h = l/2;
        arena.cellNumber = 4;               % cell number
        arena.grid{1} = [-l/2,l/2,0; h,h,0];
        arena.grid{2} = [l/2,l/2,0; h,-h,0];
        arena.grid{3} = [0,l/2,-l/2; 0,-h,-h];
        arena.grid{4} = [-l/2,0,-l/2; h,0,-h];
        arena.Grid{1} = dilationPoly(arena.grid{1},arena.ggp);
        arena.Grid{2} = dilationPoly(arena.grid{2},arena.ggp);
        arena.Grid{3} = dilationPoly(arena.grid{3},arena.ggp);
        arena.Grid{4} = dilationPoly(arena.grid{4},arena.ggp);
        InitConf{1} = [1];
        InitConf{2} = [2 3];
        InitConf{3} = [4 5 6];
        InitConf{4} = [7 8 9 10];
        switch 2
            case 1
                arena.M = [ 0.9767    0.0017    0.0017    0.0017
                      0.0033    0.9783    0.0033    0.0033
                      0.0067    0.0067    0.9817    0.0067
                      0.0133    0.0133    0.0133    0.9883];
            case 2
                arena.M = eye(arena.cellNumber,arena.cellNumber);
        end

        case '3track'
        l = 1.2;
        h = .7;
        arena.cellNumber = 3;               % cell number
        arena.grid{1} = [-l/4,3/5*l/2,l/2,l/2; h/2,h/2,h/4,-h/2];
        arena.grid{2} = [-l/8,l/4,l/2,-l/2;(2/3*h)/(l/2)*(l/2-l/8)-h/2,-h/6,-h/2,-h/2];
        arena.grid{3} = [-l/4,0,-l/2,-l/2; h/2,h/6,-h/2,h/4];
        arena.Grid{1} = dilationPoly(arena.grid{1},arena.ggp);
        arena.Grid{2} = dilationPoly(arena.grid{2},arena.ggp);
        arena.Grid{3} = dilationPoly(arena.grid{3},arena.ggp);
        InitConf{1} = [1];
        InitConf{2} = [2 3];
        InitConf{3} = [];
        switch 2
            case 2
                arena.M = eye(arena.cellNumber,arena.cellNumber);
        end
end



