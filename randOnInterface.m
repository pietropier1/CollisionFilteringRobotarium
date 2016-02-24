function p = randOnInterface(agent,origin_cell)

% Find random point on cell interface. Origin_cell is the cell the robot is coming from

p = zeros(2,1);

% switch agent.cell
%     case 1
%         X = agent.Interface{1};
%     case 2
%         X = agent.Interface{2};
%     case 3
%         X = agent.Interface{3};
%     case 4
%         
%     case 5
% end

X = agent.Interface{origin_cell};

r = 0.1 + 0.8*rand(1);

p(1,1) = X(1,1)*r + X(1,2)*(1-r);
p(2,1) = X(2,1)*r + X(2,2)*(1-r);