function [ out,opponent ] = conflictHeadings( Xo , XoG , Xn , XnG , rpz )
% function return 1 if agent are on collision heading, 0 ow
% Collisions are computed by means of the Collision Cone method with infinite horizont

makeplot = 0;
% ====== TEST ========
% close all, clear all, clc
% Xo = [-0.0150; -0.2040];
% Xn = [-0.2 0.1; -0.2 -0.05];
% XoG = [0;0];
% XnG = [0.0530 0.05; -0.2044 -0.3];
% rpz = 0.06;
% makeplot = 1;
% ====================   

rpz = 2*rpz;                                                                    % min distance is the sum of two agents protected zone
nn = size(Xn,2);                                                                % number of neighbors

ho = mod( atan2(XoG(2) - Xo(2),XoG(1) - Xo(1)) , 2*pi );                        % heading ownship
hn = mod( atan2(XnG(2,:) - Xn(2,:),XnG(1,:) - Xn(1,:)) , 2*pi );                % heading neighbors
vo = Xo + 0.25*rpz.*[cos(ho);sin(ho)];                                          % ownship velocity (dummy speed) 
vr = repmat(vo,1,nn) - 0.25*rpz.*[cos(hn);sin(hn)];                             % relative velocity vector (dummy speed)

checks = zeros(1,nn);
for nid = 1:nn
    dis = sqrt( (Xo(1)-Xn(1,nid)).^2 + (Xo(2)-Xn(2,nid)).^2 );                               % agents distance
    psi = mod( asin(rpz ./ ( dis ) ) , 2*pi );                                      
    theta = mod( atan2( (-Xo(2)+Xn(2,nid)),(-Xo(1)+Xn(1,nid))) , 2*pi);                      % relative heading

    lam = mod([(theta - psi) (theta + psi)],2*pi);                                  % tangents to protected zones

    % construct collision cone; find 3 point of the cone
    Xcc = [Xo(1) , Xo(1)+2*rpz.*cos(lam(1)) , Xo(1)+2*rpz.*cos(lam(2)) , Xo(1)];    % Collision Cone corners
    Ycc = [Xo(2) , Xo(2)+2*rpz.*sin(lam(1)) , Xo(2)+2*rpz.*sin(lam(2)) , Xo(2)];   

    ck1 = inpolygon(vr(1),vr(2),Xcc,Ycc);
    ck2 = inpolygon(vo(1),vo(2),Xcc,Ycc);
    checks(nid) = ck1 + ck2;
    
    if makeplot
        % === PLOT TEST ===
        vn = Xn + 0.25*rpz.*[cos(hn);sin(hn)];
        plot(Xo(1),Xo(2),'ko'), hold on
        plot(Xn(1,nid)+rpz*cos([0:0.1:2*pi]),Xn(2,nid)+rpz*sin([0:0.1:2*pi]),'r'), hold on
        plot(Xcc,Ycc,'g')
        plot([Xo(1),vo(1)],[Xo(2),vo(2)],'k')
        plot([Xn(1,nid),vn(1,nid)],[Xn(2,nid),vn(2,nid)],'r')
        plot([Xo(1),vr(1,nid)],[Xo(2),vr(2,nid)],'b')
        plot(XoG(1),XoG(2),'kx')
        plot(XnG(1,nid),XnG(2,nid),'rx')
        axis equal
    end

end

out = sum(checks) > 0;              % 1 if there is a collision
opponent = checks > 0;            % 1 which agent is on collision route



end

