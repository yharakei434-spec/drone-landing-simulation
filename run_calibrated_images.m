function report=run_calibrated_images()
% Sensors alone cross the estimator/controller boundary; truth is plant/render/log only.
here=fileparts(mfilename('fullpath'));out=fullfile(here,'results');
if ~isfolder(out), mkdir(out); end
c=jsondecode(fileread(fullfile(here,'base_config.json')));c.duration=40;c.camera_offset=[0;0;.25];c.width=320;c.height=240;c.focal_px=240;c.flow_sigma=.025;c.flow_delay=.08;c.marker_m=.15;c.vision_delay=.12;c.imu_delay=.02;c.height_delay=.04;c.max_age=.45;
c.gyro_sigma=.003;c.accel_sigma=.06;c.mag_sigma=.006;c.height_sigma=.012;c.corner_sigma=.35;
c.gyro_bias=[.002;-.0015;.001];c.accel_bias=[.02;-.015;.03];
c.calibration=jsondecode(fileread(fullfile(here,'calibration_model.json')));seeds=[444 555];modes={'fixed_low','calibrated_constant','calibrated_quality'};conditions={'blur','dark','occlusion'};summaries=[];
for condition=conditions
 for seed=seeds
  for mode=modes
   cc=c;cc.seed=seed;cc.mode=mode{1};cc.condition=condition{1};rng(seed);
   [s,L,im,events]=simulate(cc,cc.mode);s.seed=seed;s.condition=cc.condition;
   if isempty(summaries),summaries=s;else,summaries(end+1)=s;end
   tag=sprintf('%s_%s_seed%d',cc.condition,cc.mode,seed);writetable(L,fullfile(out,[tag '.csv']));writetable(events,fullfile(out,[tag '_vision.csv']));
   fprintf('DEGRADED %s contact=%d xy=%.4f decoded=%d/%d\n',tag,s.contact,s.xy_final_m,s.decoded_images,s.image_attempts);
   savejson(fullfile(out,'report.json'),struct('executed',true,'complete',false,'cases',summaries));
  end
 end
end
report=struct('executed',true,'complete',true,'cases',summaries,'seeds',seeds,'modes',{modes},'conditions',{conditions},'common_time_indexed_noise',true,'calibration_training_seeds',[111 222 333],'frozen_model',true,'truth_to_controller',false,'truth_to_estimator',false);
savejson(fullfile(out,'report.json'),report);
end
function [s,L,example,events]=simulate(c,name)
dt=c.dt;N=round(c.duration/dt)+1;p=[.35;-.22;1.3];v=zeros(3,1);R=eye(3);w=zeros(3,1);I=diag(c.inertia);
if strcmp(name,'initial_tilt'),R=rv([0;0;deg2rad(20)])*rv(deg2rad([6;-4;0]));end
l=c.arm_xy;q=c.yaw_torque_per_thrust;B=[ones(1,4);l l -l -l;-l l l -l;q -q q -q];f=ones(4,1)*c.mass*c.g/4;
rng(c.seed);noiseA=randn(3,4001);noiseW=randn(3,4001);noiseM=randn(3,4001);noiseZ=randn(1,4001);noiseCorners=randn(4,2,4001);
texture=uint8(imresize(40+180*rand(128),[512 512],'bilinear'));of=opticalFlowFarneback('NumPyramidLevels',4,'NumIterations',4);previous=[];prevIndex=0;flowCount=0;flowValue=[NaN;NaN];minVisualHeight=inf;
marker=generateArucoMarker(c.family,102,180);if strcmp(name,'wrong_id'),marker=generateArucoMarker(c.family,101,180);end
mworld=[1;0;.35];mworld=mworld/norm(mworld);C=diag([1 -1 -1]);acc=zeros(3,1);
% Stationary startup attitude from noisy accelerometer + magnetometer only.
a0=R'*[0;0;c.g]+c.accel_bias+c.accel_sigma*mean(randn(3,100),2);m0=R'*mworld+c.mag_sigma*mean(randn(3,100),2);g0=mean(w+c.gyro_bias+c.gyro_sigma*randn(3,100),2);zhat0=max(.1,p(3)+c.height_sigma*noiseZ(1));
e=initialize(a0,m0,zhat0,N,dt);e.bias=g0;xyint=zeros(2,1);zref=zhat0;stable=0;terminal=false;authorized=false;disarmed=false;stopcount=0;lastvision=-inf;accepted=0;stale=0;decoded=0;loss=0;example=[];
imuQueue=cell(1,N+100);imageQueue=cell(1,N+100);heightQueue=cell(1,N+100);data=zeros(N,40);eventData=zeros(0,19);imageAttempts=0;degradedAttempts=0;degradedDecoded=0;lastSigma=NaN;lastNis=NaN;lastQ=NaN;gyro=zeros(3,1);latestheight=zhat0;contact=false;flowQueue=cell(1,N+100);lastflow=-inf;acceptedNow=false;wasfresh=false;holdxy=zeros(2,1);restore=-inf;reacquired=NaN;resume=NaN;lostEver=false;
for i=1:N
 t=(i-1)*dt;acceptedNow=false;lossend=11;if ~strcmp(name,'long_loss'),lossend=3;end;if strcmp(name,'short_loss'),lossend=5;elseif strcmp(name,'permanent_loss'),lossend=inf;end;occluded=t>=3 && t<lossend;
 % Sensor simulator is the only place (besides plant/render/logger) reading truth.
 imu=struct('a',R'*(acc+[0;0;c.g])+c.accel_bias+c.accel_sigma*noiseA(:,i),'w',w+c.gyro_bias+c.gyro_sigma*noiseW(:,i),'m',R'*mworld+c.mag_sigma*noiseM(:,i));
 due=i+round(c.imu_delay/dt);imuQueue{due}=imu;
 if mod(i-1,5)==0
  pkt=struct('capture',i,'z',max(0,p(3)+c.height_sigma*noiseZ(i)));due=i+round(c.height_delay/dt);heightQueue{due}=pkt;
 end
 if mod(i-1,round(c.vision_period/dt))==0 && ~(terminal && strcmp(name,'cutoff')) && ~disarmed
  visible=~occluded;
  im=render(c,p,R,C,marker,visible,texture,i);imageAttempts=imageAttempts+1;if t>=8 && t<18,degradedAttempts=degradedAttempts+1;end;[ids,loc]=readArucoMarker(im,c.family);decoded=decoded+~isempty(ids);if t>=8 && t<18,degradedDecoded=degradedDecoded+~isempty(ids);end;idx=find(double(ids)==102,1);
  if ~isempty(idx)
   quality=image_quality(im,loc(:,:,idx));
   pkt=struct('capture',i,'corners',double(loc(:,:,idx))+c.corner_sigma*noiseCorners(:,:,i),'quality',quality);due=i+round(c.vision_delay/dt);imageQueue{due}=pkt;
   if isempty(example),example=insertShape(repmat(im,1,1,3),'Polygon',reshape(loc(:,:,idx)',1,[]),'Color','green');end
  end
 end
 if ~isempty(imuQueue{i}),u=imuQueue{i};else,u=e.lastimu;end
 % Estimator receives only timestamped noisy sensor data; no p/v/R/w.
 [e,gyro]=predict(e,u,i,c,mworld);dirty=i+1;
 if mod(i-1,10)==0 && ~disarmed
  frame=render(c,p,R,C,marker,~occluded,texture,i);fl=estimateFlow(of,frame);
  if ~isempty(previous)
   [vel,quality]=flow_velocity(fl,previous,frame,e.Rhist(:,:,prevIndex),e.R,e.x(3,prevIndex),e.x(3,i),c,(i-prevIndex)*dt);
   if quality
    flowQueue{i+round(c.flow_delay/dt)}=struct('capture',round((i+prevIndex)/2),'vworld',vel);flowValue=vel;flowCount=flowCount+1;
   end
  end
  previous=frame;prevIndex=i;
  if false,imwrite(frame,fullfile(fileparts(mfilename('fullpath')),'results',[name '_flow_frame.png']));save(fullfile(fileparts(mfilename('fullpath')),'results',[name '_flow_field.mat']),'fl');end
 end

 if ~isempty(heightQueue{i})
  pkt=heightQueue{i};j=pkt.capture;latestheight=pkt.z;e.measurements{j}{end+1}=struct('H',[0 0 1 zeros(1,6)],'z',pkt.z,'V',c.height_sigma^2);dirty=min(dirty,j);
 end
 if ~isempty(flowQueue{i})
  q=flowQueue{i};j=q.capture;H=[zeros(2,3) eye(2) zeros(2,4)];
  e.measurements{j}{end+1}=struct('H',H,'z',q.vworld,'V',eye(2)*c.flow_sigma^2);dirty=min(dirty,j);lastflow=(j-1)*dt;
 end
 if dirty<=i,e=replay(e,dirty,i);end
 if ~isempty(imageQueue{i})
  pkt=imageQueue{i};j=pkt.capture;
  if (i-j)*dt>c.max_age,stale=stale+1;
  else
   % Recover camera position relative to a marker at known map origin.
   % Uses historical ESTIMATED attitude and height, never capture-time truth.
   xy=zeros(2,4);valid=true;
   for a=1:4
    ray=e.Rhist(:,:,j)*C*[(pkt.corners(a,1)-(c.width+1)/2)/c.focal_px;(pkt.corners(a,2)-(c.height+1)/2)/c.focal_px;1];
    if ray(3)>-.2,valid=false;break;end
    off=e.Rhist(:,:,j)*c.camera_offset;xy(:,a)=(e.x(3,j)+off(3))*ray(1:2)/ray(3)-off(1:2);
   end
   if valid
    z=mean(xy,2);H=[eye(2) zeros(2,7)];V=eye(2)*(.025+.03*max(.1,e.x(3,j)))^2;
    innovation=z-H*e.x(:,j);S=H*e.P(:,:,j)*H'+V;
    nis=innovation'*(S\innovation);area=polyarea(pkt.corners(:,1),pkt.corners(:,2));margin=min([pkt.corners(:,1)-1;c.width-pkt.corners(:,1);pkt.corners(:,2)-1;c.height-pkt.corners(:,2)]);
    motion=norm(e.x(4:5,j))*(i-j)*dt;geometry=std(vecnorm(diff([xy xy(:,1)],1,2)))/c.marker_m;
    quality=pkt.quality;sigma=.012;
    if strcmp(c.mode,'calibrated_constant'),sigma=sqrt(c.calibration.constant_variance);
    elseif strcmp(c.mode,'calibrated_quality'),sigma=calibrated_sigma([log1p(area) quality(1) quality(2)/255 log1p(quality(3)) log1p(nis) e.x(3,j)],c.calibration);end
    V=eye(2)*sigma^2;lastQ=min(1,.012^2/sigma^2);
    S=H*e.P(:,:,j)*H'+V;lastSigma=sigma;lastNis=nis;
    priorXY=e.x(1:2,i);

    if innovation'*(S\innovation)<100
     e.measurements{j}{end+1}=struct('H',H,'z',z,'V',V);e=replay(e,j,i);lastvision=(j-1)*dt;accepted=accepted+1;acceptedNow=true;minVisualHeight=min(minVisualHeight,e.x(3,j));
     truthXY=data(j,2:3)';if j==i,truthXY=p(1:2);end
     eventData(end+1,:)=[t (j-1)*dt e.x(3,j) area margin sigma lastQ nis innovation' (e.x(1:2,i)-priorXY)' (z-truthXY)' norm(e.x(4:5,i)) terminal quality];

    end
   end
  end
 end
 % Conditional stationary pseudo-observation after sensor-based motor disarm.
 if disarmed && latestheight<.05 && norm(gyro)<.05 && abs(norm(u.a)-c.g)<.3
  e.measurements{i}{end+1}=struct('H',[zeros(3) eye(3) zeros(3)],'z',zeros(3,1),'V',eye(3)*.01^2);e=replay(e,i,i);
 end
 x=e.x(:,i);Rh=e.R;fresh=t-lastvision<=c.max_age;tilt=acos(max(-1,min(1,Rh(3,3))));
 flowfresh=t-lastflow<.25;
 if ~fresh && ~terminal
  loss=loss+1;stable=0;authorized=false;lostEver=lostEver || t>2;
  if wasfresh,holdxy=x(1:2);xyint(:)=0;end
 end
 if fresh && ~wasfresh,restore=t;end
 if acceptedNow && lostEver && t>=lossend && isnan(reacquired),reacquired=t-lossend;end
 wasfresh=fresh;
 if acceptedNow && fresh
  if norm(x(1:2))<.12 && norm(x(4:5))<.25 && tilt<deg2rad(8),stable=stable+1;else,stable=0;end
 end
 if fresh && stable>=3,authorized=true;end
 if norm(x(1:2))>.18 || norm(x(4:5))>.35 || tilt>deg2rad(12),authorized=false;end
 descend=fresh && authorized;if descend && lostEver && t>=lossend && isnan(resume),resume=t-lossend;end
 if descend && x(3)<.22,terminal=true;end
 % Controller inputs are estimated state, corrected gyro and measurement gates only.
 goal=zeros(2,1);if ~fresh,goal=holdxy;elseif t-restore<1,goal=holdxy*max(0,1-(t-restore));end
 [cmd,zref,xyint]=control(x,Rh,gyro,zref,descend,terminal,disarmed,fresh,xyint,c,B,goal,flowfresh);
 if terminal && latestheight<.025 && abs(x(6))<.3,stopcount=stopcount+1;else,stopcount=0;end
 if stopcount>=10,disarmed=true;cmd(:)=0;end
 attErr=acos(max(-1,min(1,(trace(Rh'*R)-1)/2)))*180/pi;
 data(i,:)=[t p' x(1:3)' v' x(4:6)' attErr norm(x(1:3)-p) fresh descend terminal disarmed zref t-lastvision min(eig((e.P(:,:,i)+e.P(:,:,i)')/2)) max(f) gyro' norm(x(1:2)) contact flowfresh acceptedNow stable goal' occluded flowValue' flowCount lastSigma lastNis lastQ];
 if i==N,break;end
 f=f+(cmd-f)*(1-exp(-dt/c.motor_lag));force=zeros(3,1);if strcmp(name,'dropout') && t>=6 && t<8,force=[.25;-.15;0];end
 oldv=v;acc=R*[0;0;sum(f)]/c.mass-[0;0;c.g]-c.drag*v/c.mass+force/c.mass;
 p=p+v*dt+.5*acc*dt^2;v=v+acc*dt;wd=I\(B(2:4,:)*f-cross(w,I*w));R=R*rv((w+.5*wd*dt)*dt);w=w+wd*dt;
 if p(3)<0,p(3)=0;v(:)=0;w(:)=0;contact=true;end
 acc=(v-oldv)/dt;
end
headers={'t','x_true','y_true','z_true','x_est','y_est','z_est','vx_true','vy_true','vz_true','vx_est','vy_est','vz_est','attitude_error_deg','position_error_m','vision_fresh','descend','terminal','disarmed','zref','vision_age','cov_min_eigen','max_rotor_n','gyro_x','gyro_y','gyro_z','estimated_xy_distance','contact','flow_fresh','accepted_now','stable_frames','goal_x','goal_y','marker_occluded','image_flow_vx','image_flow_vy','flow_updates','visual_sigma','visual_nis','visual_confidence'};
L=array2table(data,'VariableNames',headers);events=array2table(eventData,'VariableNames',{'arrival_s','capture_s','height_est_m','area_px2','edge_margin_px','sigma_m','confidence','prior_nis','innovation_x','innovation_y','correction_x','correction_y','measurement_error_x','measurement_error_y','estimated_speed','terminal','exposure','contrast','sharpness'});s=struct('name',name,'contact',contact,'image_attempts',imageAttempts,'degraded_attempts',degradedAttempts,'degraded_decoded',degradedDecoded,'flow_updates',flowCount,'minimum_visual_body_height_m',minVisualHeight,'loss_displacement_m',max([0;hypot(L.x_true(L.marker_occluded>0)-L.x_true(301),L.y_true(L.marker_occluded>0)-L.y_true(301))]),'reacquire_delay_s',reacquired,'resume_delay_s',resume,'xy_final_m',norm(p(1:2)),'terminal_position_rmse_m',sqrt(mean(L.position_error_m(L.terminal>0 & L.disarmed==0).^2)),'height_final_m',p(3),'position_rmse_m',sqrt(mean(L.position_error_m.^2)),'pre_disarm_position_rmse_m',sqrt(mean(L.position_error_m(~L.disarmed).^2)),'attitude_rmse_deg',sqrt(mean(L.attitude_error_deg.^2)),'accepted_images',accepted,'decoded_images',decoded,'stale_images',stale,'descent_authorized',authorized,'loss_samples',loss,'disarmed',disarmed,'finite_filter',all(isfinite(e.x(:))) && min(L.cov_min_eigen)>-1e-9);
end
function e=initialize(a,m,z,N,dt)
b3=a/norm(a);b1=m-dot(m,b3)*b3;b1=b1/norm(b1);b2=cross(b3,b1);R=[b1 b2 b3]';
e=struct('R',R,'bias',zeros(3,1),'x',zeros(9,N),'P',zeros(9,9,N),'Rhist',zeros(3,3,N),'acc',zeros(3,N),'measurements',{cell(1,N)},'dt',dt,'lastimu',struct('a',a,'w',zeros(3,1),'m',m));
e.x(:,1)=[0;0;z;zeros(6,1)];e.P(:,:,1)=diag([.5 .5 .03 .2 .2 .1 .05 .05 .05].^2);e.Rhist(:,:,1)=R;
for j=1:N,e.measurements{j}={};end
F=eye(9);F(1:3,4:6)=eye(3)*dt;F(1:3,7:9)=-eye(3)*dt^2/2;F(4:6,7:9)=-eye(3)*dt;e.F=F;e.x0=e.x(:,1);e.P0=e.P(:,:,1);
G=[eye(3)*dt^2/2;eye(3)*dt;zeros(3)];e.Q=G*(eye(3)*.25^2)*G'+diag([zeros(1,6) ones(1,3)*1e-7])*dt;
end
function [e,gyro]=predict(e,u,i,c,mworld)
dt=e.dt;an=u.a/norm(u.a);mn=u.m/norm(u.m);corr=.1*cross(an,e.R'*[0;0;1])+.4*cross(mn,e.R'*mworld);
e.bias=e.bias-.01*corr*dt;gyro=u.w-e.bias;e.R=e.R*rv((gyro+corr)*dt);e.Rhist(:,:,i)=e.R;e.lastimu=u;
e.acc(:,i)=e.R*u.a-[0;0;c.g];if i>1,e=replay(e,i,i);end
end
function e=replay(e,j,last)
for n=j:last
 if n==1,e.x(:,1)=e.x0;e.P(:,:,1)=e.P0;end
 if n>1
  a=e.acc(:,n-1);dt=e.dt;e.x(:,n)=e.F*e.x(:,n-1)+[.5*dt^2*a;dt*a;zeros(3,1)];e.P(:,:,n)=e.F*e.P(:,:,n-1)*e.F'+e.Q;
 end
 for k=1:numel(e.measurements{n})
  m=e.measurements{n}{k};P=e.P(:,:,n);K=P*m.H'/(m.H*P*m.H'+m.V);e.x(:,n)=e.x(:,n)+K*(m.z-m.H*e.x(:,n));A=eye(9)-K*m.H;e.P(:,:,n)=A*P*A'+K*m.V*K';
 end
end
end
function [cmd,zref,xyint]=control(x,R,w,zref,descend,terminal,off,fresh,xyint,c,B,goal,flowfresh)
if descend || (terminal && strcmp(c.mode,'cutoff')),rate=.08;if terminal,rate=.04;end;zref=max(-.06,zref-rate*c.dt);end
if fresh || terminal,xyint=max(-.8,min(.8,xyint+(x(1:2)-goal)*c.dt));end;axy=-1.3*(x(1:2)-goal)-1.9*x(4:5)-.35*xyint;if ~fresh && ~terminal && ~flowfresh,axy(:)=0;end;if norm(axy)>1.2,axy=axy*1.2/norm(axy);end
az=4*(zref-x(3))-3*x(6);av=[axy;az+c.g];b3=av/norm(av);b2=cross(b3,[1;0;0]);b2=b2/norm(b2);Rd=[cross(b2,b3) b2 b3];E=.5*(Rd'*R-R'*Rd);er=[E(3,2);E(1,3);E(2,1)];
tau=-[.32;.32;.20].*er-[.15;.15;.16].*w;T=c.mass*dot(av,R(:,3));cmd=min(8,max(0,B\[T;tau]));if off,cmd(:)=0;end
end
function plotlogs(logs,names,out)
f=figure('Visible','off','Color','w','Position',[100 100 1100 750]);cl=onCleanup(@() close(f));colors=[.25 .25 .25;0 .35 .7;.15 .55 .2;.65 .2 .1;.5 .15 .65];
subplot(2,2,1);hold on;for k=1:numel(names),L=logs{k};plot(L.t,hypot(L.x_true,L.y_true),'Color',colors(k,:),'LineWidth',1.4);end;grid on;xlabel('time (s)');ylabel('distance to marker (m)');legend(names,'Interpreter','none');title('Actual position: evaluation only');
subplot(2,2,2);hold on;for k=1:numel(names),L=logs{k};plot(L.t,L.z_true,'Color',colors(k,:),'LineWidth',1.4);end;grid on;xlabel('time (s)');ylabel('height (m)');title('Hold height, then resume approach');
L=logs{3};subplot(2,2,3);plot(L.t,L.zref,'Color',colors(2,:),'LineWidth',1.4);hold on;stairs(L.t,L.vision_fresh,'Color',colors(4,:));stairs(L.t,L.descend,':','Color',colors(5,:),'LineWidth',1.3);grid on;xlabel('time (s)');legend('height reference','fresh ArUco','descent');title('Long loss 3-11s: reacquisition gate');
subplot(2,2,4);plot(L.x_true,L.y_true,'Color',colors(3,:),'LineWidth',1.5);hold on;plot(L.x_est,L.y_est,'--','Color',colors(2,:));axis equal;grid on;xlabel('x (m)');ylabel('y (m)');legend('truth: evaluation','estimate: control');title('Long-loss recovery path');
st=sgtitle('SYNTHETIC: Farneback image flow + continuous terminal ArUco','Color','k');erc_plot_style(f);set(st,'Color','k');exportgraphics(f,fullfile(out,'image_flow_results.png'),'Resolution',140,'BackgroundColor','white');
end
function q=test(name,p),q=struct('name',name,'passed',logical(p));end
function savejson(p,s),f=fopen(p,'w');cl=onCleanup(@() fclose(f));fprintf(f,'%s',jsonencode(s,'PrettyPrint',true));end

function im=render(c,p,R,C,marker,visible,texture,i)
p=p+R*c.camera_offset;
[u,v]=meshgrid(1:c.width,1:c.height);r=R*C*[(u(:)'-(c.width+1)/2)/c.focal_px;(v(:)'-(c.height+1)/2)/c.focal_px;ones(1,numel(u))];
g=p(1:2)-p(3)*r(1:2,:)./r(3,:);tx=1+mod(g(1,:)/.008,510);ty=1+mod(g(2,:)/.008,510);
im=uint8(reshape(interp2(double(texture),tx,ty,'linear',128),c.height,c.width));
im=uint8(double(im)+randn(RandStream('mt19937ar','Seed',c.seed+10000+i),size(im)));if ~visible,return;end
h=c.marker_m/2;world=[-h h h -h;h h -h -h;0 0 0 0];q=C'*R'*(world-p);if any(q(3,:)<=0),return;end
uv=[c.focal_px*q(1,:)./q(3,:)+(c.width+1)/2;c.focal_px*q(2,:)./q(3,:)+(c.height+1)/2]';
n=size(marker,1);tf=fitgeotform2d([1 1;n 1;n n;1 n],uv,'projective');
stamp=imwarp(marker,tf,'OutputView',imref2d([c.height c.width]),'FillValues',255,'Interp','nearest');mask=imwarp(uint8(ones(size(marker))),tf,'OutputView',imref2d([c.height c.width]),'FillValues',0,'Interp','nearest')>0;im(mask)=stamp(mask);
im=degrade_image(im,c.condition,(i-1)*c.dt,c.seed+i);
if i==1001 && strcmp(c.mode,'fixed_low') && c.seed==444,imwrite(im,fullfile(fileparts(mfilename('fullpath')),'results',[c.condition '_example.png']));end
end
function R=rv(a)
th=norm(a);K=[0 -a(3) a(2);a(3) 0 -a(1);-a(2) a(1) 0];if th<1e-12,R=eye(3)+K;else,R=eye(3)+sin(th)/th*K+(1-cos(th))/th^2*K*K;end
end
