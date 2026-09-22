import json,math,hashlib
from pathlib import Path
# F78 V3: free-floating base attitude recovery using four arms as internal momentum actuators.
dt=0.002; T=12.; Ib=180.; Ia=[16.,14.,16.,14.]; Kp=1.8; Kd=5.5
# State: base attitude/velocity and arm relative velocities. External disturbance torque at t=2s.
theta=math.radians(8.); wb=0.; q=[0.]*4; qd=[0.]*4; effort=0.; peak=abs(theta); samples=[]
for k in range(int(T/dt)):
 t=k*dt
 ext=35.0 if 2.0<=t<2.10 else 0.0
 # desired base torque; arms exchange angular momentum with base, distributed by inertia.
 tau_cmd=-Kp*theta-Kd*wb
 weights=[x/sum(Ia) for x in Ia]
 arm_tau=[-tau_cmd*w for w in weights]
 # arm 0 disabled 5-6s; redistribute its requested torque over remaining arms
 if 5.0<=t<6.0:
  lost=arm_tau[0]; arm_tau[0]=0.; denom=sum(Ia[1:])
  for j in range(1,4): arm_tau[j]+=lost*Ia[j]/denom
 base_tau=-sum(arm_tau)+ext
 ab=base_tau/Ib
 wb+=ab*dt; theta+=wb*dt
 for j in range(4): qd[j]+=arm_tau[j]/Ia[j]*dt; q[j]+=qd[j]*dt
 peak=max(peak,abs(theta)); effort+=sum(x*x for x in arm_tau)*dt
 if k%100==0:samples.append([round(t,3),theta,wb,*q,*qd])
stable=abs(theta)<math.radians(1) and abs(wb)<0.01
r={"schema":"f78-multiarm-recovery-v3","simulation":"REAL_NUMERICAL_CONTROL_SIMULATION","model":"free_floating_base_four_internal_momentum_arms","controller":{"kp":Kp,"kd":Kd},"initial_base_attitude_deg":8.0,"disturbance":{"torque_Nm":35.0,"start_s":2.0,"duration_s":0.10},"degraded_case":{"arm":0,"disabled_start_s":5.0,"disabled_end_s":6.0,"redistribution":True},"metrics":{"stable":stable,"final_base_attitude_deg":math.degrees(theta),"final_base_rate_rad_s":wb,"peak_base_attitude_deg":math.degrees(peak),"control_effort":effort},"samples":samples,"transfer_note":"Uses the F77 lesson: closed-loop recovery plus perturbation/degraded-case testing; controller gains are not transferred from F77 because terrestrial and free-floating dynamics differ.","claim_scope":"Reduced planar numerical control simulation only; not full 3D ARCHITECTON, contact, HIL or physical validation."}
Path("out").mkdir(exist_ok=True); raw=json.dumps(r,sort_keys=True).encode(); r["result_sha256"]=hashlib.sha256(raw).hexdigest(); Path("out/f78-multiarm-recovery-v3.json").write_text(json.dumps(r,indent=2)+"\n"); print(json.dumps(r["metrics"])); raise SystemExit(0 if stable else 2)
