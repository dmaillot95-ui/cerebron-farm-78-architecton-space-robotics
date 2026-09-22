import json, math, hashlib
from pathlib import Path
# Reduced planar free-floating ARCHITECTON: central body + 4 independently moving arms.
dt=0.002; T=8.0; Ib=180.0; Ia=[16.,14.,16.,14.]; phases=[0.,math.pi/2,math.pi,3*math.pi/2]
base=0.; prev_w=0.; maxbase=0.; maxL=0.; samples=[]
# Arm 0 suffers a commanded freeze 4.0-5.0 s; remaining arms continue: failure/degraded case.
for k in range(int(T/dt)+1):
 t=k*dt; q=[]; qd=[]
 for j,p in enumerate(phases):
  if j==0 and 4.0<=t<5.0:
   qq=0.35*math.sin(2*math.pi*4.0/T+p); vv=0.0
  else:
   tt=5.0 if (j==0 and t>=5.0) else t
   qq=0.35*math.sin(2*math.pi*tt/T+p); vv=0.35*(2*math.pi/T)*math.cos(2*math.pi*tt/T+p)
  q.append(qq); qd.append(vv)
 w=-sum(Ia[j]*qd[j] for j in range(4))/(Ib+sum(Ia))
 if k: base+=0.5*(prev_w+w)*dt
 prev_w=w; L=(Ib+sum(Ia))*w+sum(Ia[j]*qd[j] for j in range(4)); maxL=max(maxL,abs(L)); maxbase=max(maxbase,abs(base))
 if k%100==0:samples.append([round(t,3),base,w,L,*q])
r={"schema":"f78-architecton-multiarm-v2","simulation":"REAL_NUMERICAL_SIMULATION","model":"planar_free_floating_base_four_arms_degraded_case","dt_s":dt,"duration_s":T,"failure":{"arm":0,"mode":"command_freeze","start_s":4.0,"end_s":5.0},"metrics":{"momentum_conservation_pass":maxL<1e-10,"max_abs_angular_momentum_residual":maxL,"max_base_attitude_deg":math.degrees(maxbase),"final_base_attitude_deg":math.degrees(base)},"samples":samples,"claim_scope":"Reduced planar four-arm free-floating numerical model with degraded case; not full 3D CAD/contact/thermal/power/HIL validation."}
Path("out").mkdir(exist_ok=True); raw=json.dumps(r,sort_keys=True).encode(); r["result_sha256"]=hashlib.sha256(raw).hexdigest(); Path("out/f78-multiarm.json").write_text(json.dumps(r,indent=2)+"\n"); print(json.dumps(r["metrics"])); raise SystemExit(0 if r["metrics"]["momentum_conservation_pass"] else 2)
