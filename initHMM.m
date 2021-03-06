function [P] = initHMM(arena)

% Produce the Markov transition matrix between cells. 

M = arena.cellNumber;
% eps=min(arena.Dlt)*0.2;
eps = 0.04;

D=arena.Dlt';
% mubar=1/M.*ones(M,1);

P=zeros(M,M);
for i=1:M-1;
    P(i,i)=eps/D(i);
    P(i+1,i)=1-P(i,i);
end;
P(M,M)=eps/D(M);
P(1,M)=1-P(M,M);


end

