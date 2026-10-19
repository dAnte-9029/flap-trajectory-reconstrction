#!/usr/bin/env python3
"""Readable numerical summaries; no trajectory fitting or threshold selection."""
from stage02_5_common import *

def main():
    account=pd.read_csv(OUT/'ledger.csv');phaseinfo=pd.read_csv(OUT/'phase_projection.csv');rough=pd.read_csv(OUT/'roughness.csv');rows=[];phase_rows=[];biasrows=[];correlations=[]
    for seg in ['A','B']:
        for sensor in ['accel','gyro']:
            d,e=joined(seg,sensor);held=d.holdout.astype(bool).values;valid=d.valid.astype(bool).values
            for split,mask in [('fit',valid&~held),('held',valid&held)]:
                a=account[(account.segment==seg)&(account.sensor==sensor)&(account.split==split)&(account.axis=='vector')];allrow=a[a.group=='all'].iloc[0]
                c=a[a.group=='cycle'];sec=a[a.group=='second'];sse=allrow.total_mse*allrow.n
                rows.append(dict(segment=seg,sensor=sensor,split=split,cycle_rmse_median=float(c.total_rmse.median()),cycle_rmse_p90=float(c.total_rmse.quantile(.9)),cycle_rmse_max=float(c.total_rmse.max()),max_cycle_sse_share=float(max(c.total_mse*c.n)/sse),max_second_sse_share=float(max(sec.total_mse*sec.n)/sse),gp_replacement_worse_cycles=int(sum(c.gp_replacement_mse_change>0)),n_cycles=len(c),c12_normalized_inner_product=float(allrow.c12/(2*np.sqrt(allrow.e1_mse*allrow.e2_mse)))))
                for term in ['e1','e2','e3','total','slow_error']:
                    value=e[term][mask];mean=np.mean(value,axis=0);biasrows.append(dict(segment=seg,sensor=sensor,split=split,term=term,mean_norm=float(np.linalg.norm(mean)),mean_square=float(sum(mean**2)),mse=float(np.mean(np.sum(value**2,axis=1)))))
            for term in ['e1','e2','e3','total','slow_error']:
                p=phaseinfo[(phaseinfo.segment==seg)&(phaseinfo.sensor==sensor)&(phaseinfo.term==term)]
                phase_rows.append(dict(segment=seg,sensor=sensor,term=term,held_mse=float(p.held_mse.sum()),descriptive_phase_reduction_fraction=float(p.held_descriptive_phase_reduction.sum()/p.held_mse.sum()),train_projection_held_mse_change=float((p.train_projection_held_mse-p.held_mse).sum())))
            samples=pd.read_csv(OUT/f'samples/{seg}_{sensor}.csv');h=samples.holdout.astype(bool)
            correlations.append(dict(segment=seg,sensor=sensor,held_error_distance_corr=float(np.corrcoef(np.linalg.norm(e['total'][h],axis=1),samples.nearest_retained_s[h])[0,1]),held_e2_distance_corr=float(np.corrcoef(np.linalg.norm(e['e2'][h],axis=1),samples.nearest_retained_s[h])[0,1])))
    for name,rs in [('cycle_summary',rows),('phase_summary',phase_rows),('mean_bias',biasrows),('distance_correlation',correlations)]:pd.DataFrame(rs).to_csv(OUT/f'{name}.csv',index=False)
    # A/B ledger difference is another algebraic comparison, not sensor causation.
    pair=account[(account.group=='all')&(account.axis=='vector')&(account.split=='held')]
    delta=[]
    for sensor in ['accel','gyro']:
        a=pair[(pair.segment=='A')&(pair.sensor==sensor)].iloc[0];b=pair[(pair.segment=='B')&(pair.sensor==sensor)].iloc[0]
        delta.append(dict(sensor=sensor,**{k:float(b[k]-a[k]) for k in ['total_mse','e1_mse','e2_mse','e3_mse','c12','c13','c23','gp_replacement_mse_change','physical_restoration_mse_change']}))
    pd.DataFrame(delta).to_csv(OUT/'AB_difference.csv',index=False)
    print(pd.DataFrame(rows).to_string(index=False));print(pd.DataFrame(phase_rows).to_string(index=False));print(pd.DataFrame(correlations).to_string(index=False))
if __name__=='__main__':main()
