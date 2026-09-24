import json, math, hashlib
from pathlib import Path

# F78 V4: preserve V3 as negative evidence and test a separately defined,
# analytically tuned PD controller on the same degraded case plus holdouts.
DT=0.002
T=12.0
IB=180.0
IA=[16.0,14.0,16.0,14.0]
STABLE_ATT_DEG=1.0
STABLE_RATE=0.01

V3={"kp":1.8,"kd":5.5,"label":"V3_BASELINE"}
# Analytic design point: wn=0.5 rad/s, zeta=0.9 for the reduced base model.
WN=0.5
ZETA=0.9
V4={"kp":IB*WN*WN,"kd":2.0*ZETA*IB*WN,"label":"V4_ANALYTIC_PD"}

def run_case(controller, initial_deg=8.0, disturbance_nm=35.0,
             disturbance_start=2.0, disturbance_duration=0.10,
             disabled_arm=0, disabled_start=5.0, disabled_end=6.0):
    kp=controller["kp"]; kd=controller["kd"]
    theta=math.radians(initial_deg); wb=0.0
    q=[0.0]*4; qd=[0.0]*4
    effort=0.0; peak=abs(theta); max_arm_tau=0.0; max_arm_rate=0.0
    for k in range(int(T/DT)):
        t=k*DT
        ext=disturbance_nm if disturbance_start <= t < disturbance_start+disturbance_duration else 0.0
        tau_cmd=-kp*theta-kd*wb
        weights=[x/sum(IA) for x in IA]
        arm_tau=[-tau_cmd*w for w in weights]
        if disabled_start <= t < disabled_end:
            lost=arm_tau[disabled_arm]
            arm_tau[disabled_arm]=0.0
            denom=sum(IA[j] for j in range(4) if j != disabled_arm)
            for j in range(4):
                if j != disabled_arm:
                    arm_tau[j]+=lost*IA[j]/denom
        base_tau=-sum(arm_tau)+ext
        wb+=(base_tau/IB)*DT
        theta+=wb*DT
        for j in range(4):
            qd[j]+=(arm_tau[j]/IA[j])*DT
            q[j]+=qd[j]*DT
            max_arm_tau=max(max_arm_tau,abs(arm_tau[j]))
            max_arm_rate=max(max_arm_rate,abs(qd[j]))
        peak=max(peak,abs(theta))
        effort+=sum(x*x for x in arm_tau)*DT
    final_deg=math.degrees(theta)
    stable=abs(final_deg)<STABLE_ATT_DEG and abs(wb)<STABLE_RATE
    return {
        "stable":stable,
        "final_base_attitude_deg":final_deg,
        "final_base_rate_rad_s":wb,
        "peak_base_attitude_deg":math.degrees(peak),
        "control_effort":effort,
        "max_arm_torque_Nm":max_arm_tau,
        "max_arm_rate_rad_s":max_arm_rate
    }

reference={
    "initial_deg":8.0,
    "disturbance_nm":35.0,
    "disturbance_start":2.0,
    "disturbance_duration":0.10,
    "disabled_arm":0,
    "disabled_start":5.0,
    "disabled_end":6.0
}
holdout={
    "initial_deg":-6.0,
    "disturbance_nm":-45.0,
    "disturbance_start":3.0,
    "disturbance_duration":0.08,
    "disabled_arm":2,
    "disabled_start":6.0,
    "disabled_end":7.5
}

v3_reference=run_case(V3,**reference)
v4_reference=run_case(V4,**reference)
v4_holdout=run_case(V4,**holdout)

robustness=[]
for disturbance_nm in (20.0,35.0,50.0,70.0):
    for disabled_arm in range(4):
        case=dict(reference)
        case["disturbance_nm"]=disturbance_nm
        case["disabled_arm"]=disabled_arm
        metrics=run_case(V4,**case)
        robustness.append({"disturbance_nm":disturbance_nm,"disabled_arm":disabled_arm,"metrics":metrics})

all_robust=all(x["metrics"]["stable"] for x in robustness)
passed=(not v3_reference["stable"]) and v4_reference["stable"] and v4_holdout["stable"] and all_robust

out={
    "schema":"f78-multiarm-recovery-v4",
    "simulation":"REAL_NUMERICAL_CONTROL_SIMULATION",
    "model":"reduced_planar_free_floating_base_four_internal_momentum_arms",
    "runtime_changed":False,
    "v3_preserved":True,
    "controller_design":{
        "method":"analytic_second_order_target_on_reduced_model",
        "wn_rad_s":WN,
        "zeta":ZETA,
        "v3":V3,
        "v4":V4
    },
    "acceptance":{
        "abs_final_attitude_deg_lt":STABLE_ATT_DEG,
        "abs_final_rate_rad_s_lt":STABLE_RATE,
        "requires_v3_reference_fail":True,
        "requires_v4_reference_pass":True,
        "requires_holdout_pass":True,
        "requires_all_robustness_cases_pass":True
    },
    "v3_reference":v3_reference,
    "v4_reference":v4_reference,
    "v4_holdout":v4_holdout,
    "robustness_cases":robustness,
    "robustness_pass_count":sum(1 for x in robustness if x["metrics"]["stable"]),
    "robustness_total":len(robustness),
    "passed":passed,
    "claim_scope":"Reduced planar numerical control simulation only. V4 demonstrates improved numerical recovery in this model and tested perturbations; it is not 3D, contact, actuator-saturation, HIL, hardware, or physical validation."
}
Path("out").mkdir(exist_ok=True)
raw=json.dumps(out,sort_keys=True,separators=(",",":")).encode()
out["result_sha256"]=hashlib.sha256(raw).hexdigest()
Path("out/f78-multiarm-recovery-v4.json").write_text(json.dumps(out,indent=2)+"\n",encoding="utf-8")
print(json.dumps({
    "passed":passed,
    "v3_reference":v3_reference,
    "v4_reference":v4_reference,
    "v4_holdout":v4_holdout,
    "robustness_pass_count":out["robustness_pass_count"],
    "robustness_total":out["robustness_total"],
    "result_sha256":out["result_sha256"]
},sort_keys=True))
raise SystemExit(0 if passed else 2)
