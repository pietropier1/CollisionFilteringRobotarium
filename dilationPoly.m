function [ vert_dil ] = dilationPoly( vert , dsp )
% dilate polygon by translating edges. Creates a new set of vertices corresponding to the polygon obtained by translation of 
% orginal edges by ammount DSP. Translation are performed towards the center of the polygon.

vert_exp = [vert(1,:) vert(1,1);vert(2,:),vert(2,1)];   % concatenate last vertex to list of vertices
n_vert = size(vert,2);                                  % number of vertices

% calculate slope, angle of inclination and intercept of edges
slope = zeros(1,n_vert);
alpha = zeros(1,n_vert);
for aa = 1:n_vert
    dx = vert_exp(1,aa+1)-vert_exp(1,aa) + ( (vert_exp(1,aa+1)-vert_exp(1,aa))==0)*eps;
    slope(aa) = (vert_exp(2,aa+1)-vert_exp(2,aa)) /dx;
    alpha(aa) = atan2(vert_exp(2,aa+1)-vert_exp(2,aa) , dx);
end
inter = vert(2,:) - slope.*vert(1,:);

inter_p = inter - dsp./cos(alpha);          % new intercepts for translated edges
inter_p = [inter_p,inter_p(1)];             % concatenate first value at the end
slope = [slope,slope(1)];                   % concatenate first value at the end

% compute new edges intersections
xi = zeros(1,n_vert);
yi = zeros(1,n_vert);
for vv = 1:n_vert
    xi(vv) = (inter_p(vv) - inter_p(vv+1)) / (slope(vv+1)-slope(vv));
    if abs(slope(vv)) < 2e10
        yi(vv) = xi(vv) * slope(vv) + inter_p(vv);
    else
        yi(vv) = xi(vv) * slope(vv+1) + inter_p(vv+1);
    end
end

vert_dil = [xi;yi];

% ===  plot === 
if 0
plot(vert_exp(1,:),vert_exp(2,:)); axis([-3 5 -3 5]), grid on,hold on
for pp = 1:n_vert
    plot([-5,5],[-5,5] .* slope(pp) + inter_p(pp),'k--')
end
plot(xi,yi,'or')

end


end

