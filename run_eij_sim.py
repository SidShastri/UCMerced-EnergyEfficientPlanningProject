"""
run_eij_sim.py — Run the EI/J mission sim on ANY raw sweep CSV.
Produces a self-contained step-CSV that the visualization can load directly.

Usage:
  python3 run_eij_sim.py --raw sweep_v3.csv --out steps.csv
  python3 run_eij_sim.py --raw demo.csv --out demo_steps.csv --query around_wall
  python3 run_eij_sim.py --raw demo.csv --out demo_steps.csv --slowdown 30

The step-CSV has one row per 5% replan step for THREE policies stacked
(learned / baseline_rrtstar / baseline_rrtconnect), with a 'mode' column,
plus every operator parameter the visualization inspects.
"""
import argparse, sys
import numpy as np, pandas as pd
from sklearn.ensemble import GradientBoostingRegressor
import warnings; warnings.filterwarnings('ignore')

TIER_MAP={'RRTConnect':'fast','LazyRRT':'fast','RRT':'fast','pRRT':'medium',
          'TRRT':'medium','InformedRRTstar':'slow','RRTstar':'slow'}

def build_lookup(raw):
    g=['Query','Start_Index','Remaining_Path_Dist','Straight_Line_Dist',
       'Start_X','Start_Y','Start_Z','Planner','Config_Resolution','Config_Range',
       'Config_GoalBias','Thread_Count','Simplify_Mode']
    agg=raw.groupby(g,as_index=False).agg(
        exec_time=('Exec_Time_Sec','mean'),
        exec_time_p90=('Exec_Time_Sec',lambda x:x.quantile(0.9)),
        solve_energy=('Batch_Solve_Energy_Joules','mean'),
        path_length=('Path_Length_Simp','mean'),
        smoothness=('Path_Smoothness_Simp','mean'),
        waypoints=('Waypoint_Count_Simp','mean'),
        clear_min=('Clearance_Min_Simp','mean'),
        simplify_energy=('Simplify_Energy_Joules','mean'),
        collision_checks=('Collision_Checks','mean'))
    agg['tier']=agg.Planner.map(TIER_MAP)
    return agg

def train_surrogates(agg):
    ohe=pd.get_dummies(agg['Planner'],prefix='p')
    X=pd.concat([agg[['Remaining_Path_Dist','Straight_Line_Dist','Config_Resolution',
                      'Config_Range','Thread_Count']].reset_index(drop=True),
                 ohe.reset_index(drop=True)],axis=1)
    X['geometry_ratio']=(agg.Straight_Line_Dist/agg.Remaining_Path_Dist.replace(0,np.nan)).clip(0,1).values
    models={}
    for t in ['path_length','smoothness','waypoints']:
        m=GradientBoostingRegressor(n_estimators=200,max_depth=4,learning_rate=0.05,random_state=42)
        m.fit(X.values,agg[t].values);models[t]=m
    return models,list(X.columns)

class Cfg:
    def __init__(s,slowdown):
        s.P_hover=250.;s.P_flight=180.;s.P_cpu=15.;s.speed=4.;s.cap=50000.
        s.clear_floor=0.6;s.replan=0.05;s.snap=8.;s.slowdown=slowdown
        s.wl,s.ws,s.ww=0.4,0.3,0.3;s.len_shift=0.7
        s.slow_min=0.40;s.med_min=0.20

def run(agg,models,feats,cfg,query,mode,fixed='RRTstar'):
    med={p:agg[agg.Planner==p].exec_time.median()*cfg.slowdown for p in TIER_MAP}
    p90={p:agg[agg.Planner==p].exec_time_p90.max()*cfg.slowdown for p in TIER_MAP}
    pts=(agg[agg.Query==query].groupby('Start_Index')
         .agg(x=('Start_X','first'),y=('Start_Y','first'),z=('Start_Z','first'),
              rem=('Remaining_Path_Dist','first'),sl=('Straight_Line_Dist','first')).reset_index())
    n=pts.Start_Index.max();pts['frac']=pts.Start_Index/n*0.9;pts=pts.sort_values('frac').reset_index(drop=True)
    total=pts.rem.iloc[0]
    worst=agg[agg.Query==query].groupby('Start_Index').agg(
        lw=('path_length','max'),lb=('path_length','min'),
        sw=('smoothness','max'),sb=('smoothness','min'),
        ww=('waypoints','max'),wb=('waypoints','min'))
    def geom(fr):
        f=pts.frac.values;i=np.clip(np.searchsorted(f,fr),1,len(f)-1)
        t=0 if f[i]==f[i-1] else (fr-f[i-1])/(f[i]-f[i-1])
        r=lambda c:pts[c].values[i-1]*(1-t)+pts[c].values[i]*t
        return [r('x'),r('y'),r('z')],r('rem'),r('sl'),pts.Start_Index.values[i]
    def nearest(pos):
        st=pts.set_index('Start_Index')[['x','y','z']]
        d=np.sqrt(((st.values-np.array(pos))**2).sum(1));return st.index[np.argmin(d)],d.min()
    batt=cfg.cap;fr=0.;out=[];step=0
    while fr<0.999 and batt>0:
        pos,rem,sl,_=geom(fr);bf=batt/cfg.cap
        if mode=='learned':
            push=cfg.len_shift*(1-bf);wl=cfg.wl+push*(1-cfg.wl);rem_w=1-wl
            ws=rem_w*cfg.ws/(cfg.ws+cfg.ww);ww=rem_w*cfg.ww/(cfg.ws+cfg.ww)
            allowed={'fast'} if bf<cfg.med_min else ({'fast','medium'} if bf<cfg.slow_min else {'fast','medium','slow'})
            si,_=nearest(pos);best=None;bs=-1
            for pl in TIER_MAP:
                if TIER_MAP[pl] not in allowed:continue
                seg=(rem*cfg.replan)/cfg.speed
                if p90[pl]>seg:continue
                for res in [0.0005,0.002,0.008]:
                    for rng in [0.05,0.2,0.5]:
                        for th in ([1,2,4,8] if pl=='pRRT' else [1]):
                            r=agg[(agg.Query==query)&(agg.Start_Index==si)&(agg.Planner==pl)&
                                  (agg.Config_Resolution==res)&(agg.Config_Range==rng)&
                                  (agg.Thread_Count==th)&(agg.Simplify_Mode=='max')]
                            if not len(r):continue
                            cl=r.clear_min.mean()
                            if np.isnan(cl) or cl<cfg.clear_floor:continue
                            pl_,sm_,wp_=r.path_length.mean(),r.smoothness.mean(),r.waypoints.mean()
                            wr=worst.loc[si]
                            def nm(v,b,w):s=w-b;return .5 if s<=0 else np.clip((w-v)/s,0,1)
                            q=wl*nm(pl_,wr.lb,wr.lw)+ws*nm(sm_,wr.sb,wr.sw)+ww*nm(wp_,wr.wb,wr.ww)
                            flight=(pl_/cfg.speed)*cfg.P_flight;hover=(cfg.P_hover+cfg.P_cpu)*med[pl]
                            cost=flight+hover+r.simplify_energy.mean()
                            sc=q/cost if cost>0 else 0
                            if sc>bs:bs=sc;best=(pl,res,rng,th,r)
            if best is None:pl,res,rng,th='RRTConnect',0.0005,0.05,1;src='fallback';r=None
            else:pl,res,rng,th,r=best;src='lookup'
        else:
            pl,res,rng,th=fixed,0.0005,0.05,1;src='fixed'
            si,_=nearest(pos)
            r=agg[(agg.Query==query)&(agg.Start_Index==si)&(agg.Planner==pl)&
                  (agg.Config_Resolution==res)&(agg.Config_Range==rng)&(agg.Simplify_Mode=='max')]
        if r is None or not len(r):
            si,_=nearest(pos)
            r=agg[(agg.Query==query)&(agg.Start_Index==si)&(agg.Planner==pl)&(agg.Simplify_Mode=='max')]
        pt=r.exec_time.mean()*cfg.slowdown;plen=r.path_length.mean()
        seg_d=total*cfg.replan;hover=(cfg.P_hover+cfg.P_cpu)*pt;flight=(seg_d/cfg.speed)*cfg.P_flight
        batt-=hover+flight
        out.append(dict(step=step,frac=round(fr,3),mode=mode,planner=pl,tier=TIER_MAP[pl],
            resolution=res,range=rng,threads=th,source=src,is_opt='Yes' if pl in['RRTstar','InformedRRTstar'] else 'No',
            battery_fraction=round(max(batt,0)/cfg.cap,4),plan_time_s=round(pt,5),
            hover_J=round(hover,3),flight_J=round(flight,1),cumulative_J=round(cfg.cap-max(batt,0),1),
            path_length=round(plen,1),waypoints=round(r.waypoints.mean(),1),
            smoothness=round(r.smoothness.mean(),5),clear_min=round(r.clear_min.mean(),3),
            collision_checks=int(r.collision_checks.mean())))
        fr+=cfg.replan;step+=1
    return pd.DataFrame(out)

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--raw',required=True);ap.add_argument('--out',default='steps.csv')
    ap.add_argument('--query',default='around_wall');ap.add_argument('--slowdown',type=float,default=30.)
    a=ap.parse_args()
    print(f"Loading {a.raw} ...")
    raw=pd.read_csv(a.raw)
    need=['Query','Start_Index','Planner','Config_Resolution','Exec_Time_Sec',
          'Path_Length_Simp','Clearance_Min_Simp','Waypoint_Count_Simp','Simplify_Mode']
    miss=[c for c in need if c not in raw.columns]
    if miss:print(f"ERROR: raw CSV missing columns: {miss}");sys.exit(1)
    if a.query not in raw.Query.unique():
        a.query=raw.Query.unique()[0];print(f"query not found, using '{a.query}'")
    agg=build_lookup(raw);models,feats=train_surrogates(agg)
    cfg=Cfg(a.slowdown)
    print(f"Running EI/J sim on '{a.query}' (slowdown {a.slowdown}x) ...")
    L=run(agg,models,feats,cfg,a.query,'learned')
    S=run(agg,models,feats,cfg,a.query,'baseline',fixed='RRTstar')
    C=run(agg,models,feats,cfg,a.query,'baseline',fixed='RRTConnect')
    allsteps=pd.concat([L.assign(policy='learned'),S.assign(policy='rrtstar'),
                        C.assign(policy='rrtconnect')],ignore_index=True)
    allsteps.to_csv(a.out,index=False)
    print(f"\nWrote {a.out}: {len(L)} steps x 3 policies")
    print(f"  learned  end battery {L.battery_fraction.iloc[-1]:.0%}, energy {L.cumulative_J.iloc[-1]:.0f}J")
    print(f"  rrtstar  end battery {S.battery_fraction.iloc[-1]:.0%}, energy {S.cumulative_J.iloc[-1]:.0f}J")
    print(f"  rrtconn  end battery {C.battery_fraction.iloc[-1]:.0%}, energy {C.cumulative_J.iloc[-1]:.0f}J")

if __name__=='__main__':main()
