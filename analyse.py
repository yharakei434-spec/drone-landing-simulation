from pathlib import Path
import json,hashlib,numpy as np,csv
D=Path(__file__).resolve().parent;O=D/'results';r=json.loads((O/'report.json').read_text());assert r['complete'];model=json.loads((D/'calibration_model.json').read_text());checks=[];rows=[]
def test(n,b):checks.append({'name':n,'passed':bool(b)})
test('model unchanged since before holdout tests',hashlib.sha256((D/'calibration_model.json').read_bytes()).hexdigest()==(D/'model_frozen.sha256').read_text().strip())
test('calibration/test seeds disjoint',not set(model['training_seeds'])&set(r['seeds']))
for p in model['sources']:test('training source unchanged: '+Path(p['path']).name,hashlib.sha256((D/p['path']).read_bytes()).hexdigest()==p['sha256'])
for s in r['cases']:
 tag='%s_%s_seed%d'%(s['condition'],s['name'],s['seed']);raw=np.genfromtxt(O/(tag+'.csv'),delimiter=',',names=True,dtype=None,encoding='utf-8');eigen=np.array([complex(str(v).replace('i','j')) for v in raw['cov_min_eigen']]);test(tag+': covariance eigenvalue imaginary roundoff',max(abs(eigen.imag))<1e-10);a=np.genfromtxt(O/(tag+'.csv'),delimiter=',',names=True,converters={'cov_min_eigen':lambda v: complex((v.decode() if isinstance(v,bytes) else v).replace('i','j')).real});ev=np.atleast_1d(np.genfromtxt(O/(tag+'_vision.csv'),delimiter=',',names=True))
 test(tag+': finite/PSD',np.isfinite(a['position_error_m']).all() and min(a['cov_min_eigen'])>-1e-9)
 test(tag+': no stale image descent',not np.any((a['vision_fresh']==0)&(a['descend']==1)))
 test(tag+': positive measurement sigma',np.all(np.isfinite(ev['sigma_m'])) and np.all(ev['sigma_m']>0))
 error2=ev['measurement_error_x']**2+ev['measurement_error_y']**2;radial=error2/ev['sigma_m']**2;mask=(ev['capture_s']>=8)&(ev['capture_s']<18)
 rows.append({**s,'observation_coverage_95':float(np.mean(radial<=5.991464547)),'observation_rmse_m':float(np.sqrt(np.mean(error2))),'mean_sigma_m':float(np.mean(ev['sigma_m'])),'mean_nll':float(np.mean(np.log(ev['sigma_m']**2)+.5*radial)),'accepted_observations':len(ev),'degraded_observations':int(sum(mask)),'degraded_coverage_95':float(np.mean(radial[mask]<=5.991464547)) if sum(mask) else None,'detection_rate':s['degraded_decoded']/max(1,s['degraded_attempts'])})
 if s['name']=='calibrated_constant':test(tag+': calibrated constant stays fixed',np.allclose(ev['sigma_m'],np.sqrt(model['constant_variance'])))
 if s['name']=='calibrated_quality':
  X=np.column_stack([np.log1p(ev['area_px2']),ev['exposure'],ev['contrast']/255,np.log1p(ev['sharpness']),np.log1p(ev['prior_nis']),ev['height_est_m']]);Z=np.column_stack([np.ones(len(ev)),(X-model['mean'])/model['std']]);pred=np.clip(np.exp(np.clip(Z@model['coefficients'],np.log(model['sigma_floor_m']**2),np.log(model['sigma_cap_m']**2)))*model['variance_scale'],model['sigma_floor_m']**2,model['sigma_cap_m']**2)**.5
  test(tag+': frozen model reproduces weights',np.allclose(pred,ev['sigma_m'],atol=1e-8))
stats={}
for cond in r['conditions']:
 stats[cond]={}
 for mode in r['modes']:
  rr=[x for x in rows if x['condition']==cond and x['name']==mode];stats[cond][mode]={'mean_final_distance_m':float(np.mean([x['xy_final_m'] for x in rr])),'worst_final_distance_m':max(x['xy_final_m'] for x in rr),'contacts':sum(x['contact'] for x in rr),'cases':len(rr),'mean_coverage_95':float(np.mean([x['observation_coverage_95'] for x in rr])),'mean_sigma_m':float(np.mean([x['mean_sigma_m'] for x in rr]))}
overall={mode:{'mean_final_distance_m':float(np.mean([x['xy_final_m'] for x in rows if x['name']==mode])),'contacts':sum(x['contact'] for x in rows if x['name']==mode),'cases':sum(x['name']==mode for x in rows)} for mode in r['modes']}
summary={'rows':rows,'statistics':stats,'overall':overall,'checks':checks,'checks_passed':all(x['passed'] for x in checks),'training_oof_coverage_95':model['oof_radial_95_coverage'],'note':'Two untouched test seeds; covariance coverage assumes isotropic independent errors but frames are correlated. Test outcomes were not used to refit or rescale.'};(O/'analysis.json').write_text(json.dumps(summary,indent=2));print(json.dumps({'stats':stats,'checks':len(checks),'failed':[x for x in checks if not x['passed']]}));assert summary['checks_passed']
with (O/'comparison.csv').open('w',newline='') as f:
 w=csv.writer(f,lineterminator='\n');w.writerow(['condition','seed','policy','contact','final_distance_m','coverage_95','mean_sigma_m'])
 for x in rows:w.writerow([x['condition'],x['seed'],x['name'],x['contact'],x['xy_final_m'],x['observation_coverage_95'],x['mean_sigma_m']])
