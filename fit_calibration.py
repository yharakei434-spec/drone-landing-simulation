"""Offline calibration on v8 fixed-policy seeds only; no new-test data read."""
from pathlib import Path
import numpy as np,json,hashlib
D=Path(__file__).resolve().parent;source=D/'calibration_data';parts=[];groups=[];paths=[]
for seed in [111,222,333]:
 for cond in ['blur','dark','occlusion']:
  p=source/f'{cond}_fixed_low_seed{seed}_vision.csv';a=np.genfromtxt(p,delimiter=',',names=True);parts.append(a);groups.extend([seed]*len(a));paths.append({'path':p.relative_to(D).as_posix(),'sha256':hashlib.sha256(p.read_bytes()).hexdigest()})
a=np.concatenate(parts);groups=np.array(groups);X=np.column_stack([np.log1p(a['area_px2']),a['exposure'],a['contrast']/255,np.log1p(a['sharpness']),np.log1p(a['prior_nis']),a['height_est_m']]);target=(a['measurement_error_x']**2+a['measurement_error_y']**2)/2;floor=.0005;cap=.05;y=np.log(np.maximum(target,1e-10))
def fit(mask,lam):
 mu=X[mask].mean(0);sd=np.maximum(X[mask].std(0),1e-8);Z=np.column_stack([np.ones(sum(mask)),(X[mask]-mu)/sd]);pen=np.eye(Z.shape[1])*lam;pen[0,0]=0;b=np.linalg.solve(Z.T@Z+pen,Z.T@y[mask]);return mu,sd,b
def predict(mask,model):
 mu,sd,b=model;z=np.column_stack([np.ones(sum(mask)),(X[mask]-mu)/sd]);return np.clip(np.exp(np.sum(z*b,axis=1)),floor**2,cap**2)
def scale(target,pred):return float(max(np.mean(target/pred),np.quantile(2*target/(5.991464547*pred),.95),1))
scores=[];oofs={}
for lam in [.1,1,10,100]:
 oof=np.zeros(len(a));loss=[]
 for seed in [111,222,333]:
  train=groups!=seed;test=~train;model=fit(train,lam);p=predict(test,model);s=scale(target[train],predict(train,model));loss.extend(np.log(s*p)+target[test]/(s*p));oof[test]=p
 scores.append({'ridge':lam,'cv_nll':float(np.mean(loss))});oofs[lam]=oof
best=min(scores,key=lambda x:x['cv_nll'])['ridge'];mu,sd,b=fit(np.ones(len(a),bool),best);inflation=scale(target,oofs[best]);constant=float(np.mean(target));constant_oof=np.array([np.mean(target[groups!=g]) for g in groups]);constant_scale=scale(target,constant_oof)
model={'date':'2026-10-27','training_seeds':[111,222,333],'test_seeds':[444,555],'training_policy':'v8 fixed_low only','features':['log1p(area)','exposure','contrast/255','log1p(sharpness)','log1p(prior_nis)','estimated_height'],'mean':mu.tolist(),'std':sd.tolist(),'coefficients':b.tolist(),'ridge':best,'variance_scale':inflation,'sigma_floor_m':floor,'sigma_cap_m':cap,'constant_variance':constant*constant_scale,'cv_scores':scores,'samples':len(a),'sources':paths,'oof_radial_95_coverage':float(np.mean(2*target<=5.991464547*inflation*oofs[best])),'note':'Offline synthetic ground-truth errors calibrate covariance only; no test errors or online truth inputs. CV grouped by seed; frames are correlated.'}
p=D/'calibration_model.json';p.write_text(json.dumps(model,indent=2));(D/'model_frozen.sha256').write_text(hashlib.sha256(p.read_bytes()).hexdigest()+'\n');print(json.dumps({k:model[k] for k in ['samples','ridge','variance_scale','constant_variance','oof_radial_95_coverage']}))
