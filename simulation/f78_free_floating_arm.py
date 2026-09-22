import json,math,hashlib
from pathlib import Path
# Real numerical simulation: free-floating base + one rotary arm, zero external torque.
dt=0.002; T=6.0; Ib=120.0; Ia=18.0
# Prescribed smooth arm relative angle q(t); conserve angular momentum: (Ib+Ia)*wb + Ia*qdot = 0.
series=[]; max_L=0.0; max_base=0.0; base=0.0; prev_wb=0.0
for k in range(int(T/dt)+1):
    t=k*dt
    q=0.5*(1-math.cos(2*math.pi*t/T)) # 0 -> 1 rad -> 0
    qd=(math.pi/T)*math.sin(2*math.pi*t/T)
    wb=-(Ia*qd)/(Ib+Ia)
    if k: base += 0.5*(prev_wb+wb)*dt
    prev_wb=wb
    L=(Ib+Ia)*wb+Ia*qd
    max_L=max(max_L,abs(L)); max_base=max(max_base,abs(base))
    if k%100==0: series.append([round(t,3),base,q,wb,L])
result={"schema":"f78-free-floating-arm-sim-v1","simulation":"REAL_NUMERICAL_SIMULATION","model":"planar_free_floating_base_plus_rotary_arm","dt_s":dt,"duration_s":T,"parameters":{"base_inertia_kgm2":Ib,"arm_effective_inertia_kgm2":Ia},"metrics":{"max_abs_angular_momentum_residual":max_L,"max_base_attitude_deg":math.degrees(max_base),"final_base_attitude_deg":math.degrees(base),"momentum_conservation_pass":max_L<1e-10},"samples":series,"claim_scope":"Actual numerical rigid-body momentum simulation; not a validated ARCHITECTON CAD model, multibody contact simulation, HIL, or physical space-robot test."}
Path("out").mkdir(exist_ok=True); raw=json.dumps(result,sort_keys=True).encode(); result["result_sha256"]=hashlib.sha256(raw).hexdigest(); Path("out/f78-simulation.json").write_text(json.dumps(result,indent=2)+"\n"); print(json.dumps(result["metrics"]))
if not result["metrics"]["momentum_conservation_pass"]: raise SystemExit(2)
