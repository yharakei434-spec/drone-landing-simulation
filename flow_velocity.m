function [velocity,good]=flow_velocity(fl,prev,current,R0,R1,z0,z1,c,dt)
% Only decoded pixel flow and ESTIMATED camera geometry. No plant state.
C=diag([1 -1 -1]);[u,v]=meshgrid(15:8:c.width-15,15:8:c.height-15);idx=sub2ind(size(prev),v(:),u(:));
[gx,gy]=gradient(double(prev));valid=hypot(gx(idx),gy(idx))>4;
x0=u(:)';y0=v(:)';x1=x0+double(fl.Vx(idx))';y1=y0+double(fl.Vy(idx))';
valid=valid' & x1>2 & x1<c.width-1 & y1>2 & y1<c.height-1;
r0=R0*C*[(x0-(c.width+1)/2)/c.focal_px;(y0-(c.height+1)/2)/c.focal_px;ones(size(x0))];
r1=R1*C*[(x1-(c.width+1)/2)/c.focal_px;(y1-(c.height+1)/2)/c.focal_px;ones(size(x1))];
o0=R0*c.camera_offset;o1=R1*c.camera_offset;
a0=-(z0+o0(3))*r0(1:2,:)./r0(3,:);a1=-(z1+o1(3))*r1(1:2,:)./r1(3,:);
samples=(a0-a1-(o1(1:2)-o0(1:2)))/dt;valid=valid & r0(3,:)<-.2 & r1(3,:)<-.2;
samples=samples(:,valid);good=size(samples,2)>40;velocity=[0;0];if ~good,return;end
med=median(samples,2);keep=vecnorm(samples-med)<.20;samples=samples(:,keep);good=size(samples,2)>30;
if good,velocity=median(samples,2);good=norm(velocity)<2 && median(vecnorm(samples-velocity))<.12;end
end
