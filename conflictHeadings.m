function [ out ] = conflictHeadings( Xo , XoG , Xn , XnG , rpz )
% function return 1 if agent are on collision heading, 0 ow
% Collisions are computed by means of the Collision Cone method

makeplot = 0;
% % ====== TEST ========
% close all, clear all, clc
% Xo = [-0.0150; -0.2040];
% Xn = [-0.1363; -0.16];
% XoG = [0;0];
% XnG = [0.0530; -0.2044];
% rpz = 0.06;
% makeplot = 1;
% % ====================   

rpz = 2*rpz;                                                                    % min distance is the sum of two agents protected zone
nn = size(Xn,2);                                                                % number of neighbors

ho = mod( atan2(XoG(2) - Xo(2),XoG(1) - Xo(1)) , 2*pi );                        % heading ownship
hn = mod( atan2(XnG(2,:) - Xn(2,:),XnG(1,:) - Xn(1,:)) , 2*pi );                        % heading neighbors
vo = Xo + 0.25*rpz.*[cos(ho);sin(ho)];

vr = repmat(vo,1,nn) - 0.25*rpz.*[cos(hn);sin(hn)];            % relative velocity vector

dis = sqrt( (Xo(1)-Xn(1,:)).^2 + (Xo(2)-Xn(2,:)).^2 );                               % agents distance
psi = mod( asin(rpz ./ ( dis ) ) , 2*pi );                                      
theta = mod( atan2( (-Xo(2)+Xn(2,:)),(-Xo(1)+Xn(1,:))) , 2*pi);                      % relative heading

checks = 0;
for nid = 1:nn
    lam = mod([(theta(nid) - psi(nid)) (theta(nid) + psi(nid))],2*pi);                                  % tangents to protected zones

    % construct collision cone; find 3 point of the cone
    Xcc = [Xo(1) , Xo(1)+2*rpz.*cos(lam(1)) , Xo(1)+2*rpz.*cos(lam(2)) , Xo(1)];    % Collision Cone corners
    Ycc = [Xo(2) , Xo(2)+2*rpz.*sin(lam(1)) , Xo(2)+2*rpz.*sin(lam(2)) , Xo(2)];   

    ck1 = inpolygon(vr(1),vr(2),Xcc,Ycc);
    ck2 = inpolygon(vo(1),vo(2),Xcc,Ycc);
    checks = checks + ck1 + ck2;

end

out = checks > 0;

if makeplot
    % === PLOT TEST ===
    vn = Xn + 0.25*rpz.*[cos(hn);sin(hn)];
    figure
    plot(Xo(1),Xo(2),'ko'), hold on
    plot(Xn(1,:)+rpz*cos([0:0.1:2*pi]),Xn(2,:)+rpz*sin([0:0.1:2*pi]),'r'), hold on
    plot(Xcc,Ycc,'g')
    plot([Xo(1),vo(1)],[Xo(2),vo(2)],'k')
    plot([Xn(1),vn(1)],[Xn(2),vn(2)],'r')
    plot([Xo(1),vr(1)],[Xo(2),vr(2)],'b')
    plot(XoG(1),XoG(2),'kx')
    plot(XnG(1),XnG(2),'rx')
    axis equal
end

end

