#!/usr/bin/env python3
"""Readable evidence plots, all from native timestamps and saved diagnostics."""
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from stage02_5_common import *

COL={'e1':'tab:blue','e2':'tab:orange','e3':'tab:green','total':'black'}

def finish(fig,path):
    for ax in fig.axes:ax.grid(alpha=.2)
    fig.tight_layout();fig.savefig(path,dpi=145);plt.close(fig)


def main():
    folder=OUT/'figures';folder.mkdir(exist_ok=True);account=pd.read_csv(OUT/'ledger.csv');spec=pd.read_csv(OUT/'spectra.csv');white=pd.read_csv(OUT/'whitening.csv')
    for seg in ['A','B']:
        for sensor in ['accel','gyro']:
            d=pd.read_csv(OUT/f'samples/{seg}_{sensor}.csv');t=d.t.values;t0=t[0];cy=cycles(seg);held=d.holdout.astype(bool).values;unit='m/s²' if sensor=='accel' else 'rad/s'
            fig,axes=plt.subplots(3,2,figsize=(15,10))
            for j in range(3):
                ax=axes[j,0];near=t<t0+3
                ax.plot(t[near]-t0,d.loc[near,f'remaining{j}'],'k.',ms=3,label='remaining observation')
                ax.plot(t[near]-t0,d.loc[near,f'T{j}'],'--',label='training trend T')
                ax.plot(t[near]-t0,d.loc[near,f'S{j}'],label='slow GP prediction S')
                for c in np.unique(d.cycle[held]):
                    row=cy.iloc[c]
                    if row.lo<t0+3 and row.hi>t0:ax.axvspan(max(0,row.lo-t0),min(3,row.hi-t0),color='red',alpha=.09)
                ax.set_title(f'{seg} {sensor}-{"xyz"[j]}: first 3s; red = held wingbeat');ax.set_ylabel(unit)
                for k in COL:axes[j,1].plot(t-t0,d[f'{k}{j}'],lw=.65,alpha=.8,color=COL[k],label=k)
                axes[j,1].set_title('Full native residual ledger');axes[j,1].set_ylabel(unit)
                for a in axes[j]:a.set_xlabel('time from first core sample (s)')
            axes[0,0].legend(fontsize=8);axes[0,1].legend(fontsize=8);finish(fig,folder/f'{seg}_{sensor}_time.png')
            fig,axes=plt.subplots(3,2,figsize=(15,10));phase_mod=np.mod(d.phase_rad,2*np.pi);bins=np.linspace(0,2*np.pi,25);centers=(bins[1:]+bins[:-1])/2
            for j in range(3):
                for col,(split,mask) in enumerate([('fit',~held),('held',held)]):
                    ax=axes[j,col];ax.scatter(phase_mod[mask],d.loc[mask,f'total{j}'],s=2,c='gray',alpha=.2,label='total samples')
                    for k in COL:
                        means=[d.loc[mask&(phase_mod>=l)&(phase_mod<r),f'{k}{j}'].mean() for l,r in zip(bins[:-1],bins[1:])]
                        ax.plot(centers,means,color=COL[k],label=k+' bin mean')
                    ax.set_title(f'{seg} {sensor}-{"xyz"[j]} {split}: descriptive phase fold');ax.set_xlabel('mechanical phase (rad)');ax.set_ylabel(unit)
            axes[0,0].legend(fontsize=7);finish(fig,folder/f'{seg}_{sensor}_phase.png')
            fig,axes=plt.subplots(3,2,figsize=(15,10))
            for j in range(3):
                for col,split in enumerate(['full','held_gapped']):
                    ax=axes[j,col]
                    for k in COL:
                        v=spec[(spec.segment==seg)&(spec.sensor==sensor)&(spec.axis==j)&(spec.term==k)&(spec.split==split)]
                        ax.semilogy(v.hz,np.maximum(v.power,1e-7),color=COL[k],lw=.7,label=k)
                    ax.set_title(f'{seg} {sensor}-{"xyz"[j]}: {split} native Lomb');ax.set_xlabel('frequency (Hz)');ax.set_ylabel('normalized Lomb power (NOT PSD)');ax.set_ylim(1e-6,1)
            axes[0,0].legend(fontsize=8);finish(fig,folder/f'{seg}_{sensor}_spectrum.png')
            fig,axes=plt.subplots(3,1,figsize=(13,9))
            for j in range(3):
                a=account[(account.segment==seg)&(account.sensor==sensor)&(account.group=='cycle')&(account.axis==str(j))]
                for split,style in [('fit','.-'),('held','o')]:
                    b=a[a.split==split]
                    for k in COL:axes[j].plot(cy.mid.iloc[b.group_id].values-t0,b[f'{k}_rmse'] if k!='total' else b.total_rmse,style,color=COL[k],ms=4,alpha=.8,label=k+' '+split)
                axes[j].set_title(f'{seg} {sensor}-{"xyz"[j]}: per-cycle RMSE; dots=fit, circles=held');axes[j].set_xlabel('time from first core sample (s)');axes[j].set_ylabel(unit)
            axes[0].legend(fontsize=7,ncol=4);finish(fig,folder/f'{seg}_{sensor}_cycles.png')
    # Signed cross terms must be visible; they are not positive component shares.
    fig,axes=plt.subplots(1,2,figsize=(14,5))
    for ax,sensor in zip(axes,['accel','gyro']):
        a=account[(account.sensor==sensor)&(account.group=='all')&(account.axis=='vector')];labels=[]
        for x,(_,row) in enumerate(a.iterrows()):
            labels.append(row.segment+' '+row.split);pos=neg=0
            for k,c in zip(['e1_mse','e2_mse','e3_mse','c12','c13','c23'],['tab:blue','tab:orange','tab:green','tab:red','tab:purple','tab:brown']):
                val=row[k];bottom=pos if val>=0 else neg;ax.bar(x,val,bottom=bottom,color=c,label=k if x==0 else None)
                if val>=0:pos+=val
                else:neg+=val
            ax.plot(x,row.total_mse,'k_',ms=15)
        ax.set_xticks(range(len(labels)),labels);ax.set_title(sensor+': signed MSE ledger; black = total');ax.set_ylabel('squared sensor units');ax.legend(fontsize=8)
    finish(fig,folder/'signed_ledger.png')
    fig,axes=plt.subplots(1,2,figsize=(14,5))
    for ax,sensor in zip(axes,['accel','gyro']):
        for name in ['A','B','A_prior1']:
            w=pd.read_csv(OUT/f'{name}_{sensor}_weights.csv');w=w[w.core==1]
            for held,style in [(False,'-'),(True,'--')]:
                v=np.sort(w.white_norm[w.holdout==held]);ax.plot(v,np.arange(1,len(v)+1)/len(v),style,label=name+(' held (counterfactual)' if held else ' fit'))
        ax.axvline(3,color='black',ls=':');ax.set_xlim(0,25);ax.set_xlabel('whitened residual vector norm');ax.set_ylabel('empirical CDF');ax.set_title(sensor+' Huber threshold=3');ax.legend(fontsize=7)
    finish(fig,folder/'whitened_residuals.png')
    fig,axes=plt.subplots(1,2,figsize=(13,5));it=pd.read_csv(OUT/'interval_sensitivity.csv')
    for ax,key in zip(axes,['average_a_difference_rms','average_alpha_component_difference_rms']):
        for name in ['A_prior1','A_delay0','A_lever','A_phase16']:
            d=it[(it.name==name)&(it.kind=='restore')];ax.plot(1000*d.h,d[key],'o-',label=name)
        ax.set_title(key);ax.set_xlabel('interval length (ms)');ax.set_ylabel('m/s²' if key.startswith('average_a_') else 'rad/s²');ax.legend(fontsize=8)
    finish(fig,folder/'interval_sensitivity.png')
    fig,axes=plt.subplots(1,2,figsize=(13,4))
    for ax,seg in zip(axes,['A','B']):
        for split in ['full','held_gapped']:
            d=pd.read_csv(OUT/f'{seg}_accel_{split}_spectral_window.csv');ax.semilogy(d.hz,np.maximum(d.window_power,1e-9),label=split)
        ax.set_title(seg+' timestamp sampling window');ax.set_xlabel('frequency (Hz)');ax.set_ylabel('sampling-window power');ax.legend()
    finish(fig,folder/'sampling_window.png')
    text='<meta charset="utf-8"><title>Stage02.5 残差归因</title><h1>Stage02.5：残差账本与慢变轨迹诊断</h1><p>e1=趋势+周期−观测；e2=慢变GP预测−趋势；e3=物理完整预测−慢变预测−周期；total=e1+e2+e3。所有曲线保留原生时刻。训练与整翼拍留出是条件性预测测试，不能作为物理真值。</p><p>time图左边比较扣周期后的观测、训练趋势与慢变GP，红背景是未参与拟合的整翼拍；右边是全时段误差。phase图横轴是机械相位，散点是总残差，曲线是分箱均值，并非新的拟合模型。spectrum图用Lomb方法，右列直接用有缺口的原生留出时间，不拼接；采样窗口会产生旁瓣，峰值不能直接认定为结构模态。cycles图按完整周期比较各轴误差。signed ledger的负值是误差抵消，不能加RMSE。</p>'
    for p in sorted(folder.glob('*.png')):text+=f'<h2>{p.stem}</h2><img style="max-width:100%" src="figures/{p.name}">'
    (OUT/'index.html').write_text(text)
if __name__=='__main__':main()
