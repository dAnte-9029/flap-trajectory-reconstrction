#!/usr/bin/env python3
"""Readable standalone plots and a self-contained Chinese phase reconstruction gallery."""
import base64
import json
import subprocess
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
import numpy as np
import pandas as pd

from stage02_3A_reconstruct import ContinuousPhase, load_data, dump, sha


def main():
    cfg=json.load(open('configs/stage02_3A_phase.json'));out=Path(cfg['output']);dest=out/'figures'
    dest.mkdir(exist_ok=False)
    fp=subprocess.check_output(['fc-match','-f','%{file}','Noto Sans CJK SC'],text=True)
    font_manager.fontManager.addfont(fp)
    plt.rcParams.update({'font.family':font_manager.FontProperties(fname=fp).get_name(),
                         'font.size':11,'axes.unicode_minus':False,'svg.fonttype':'path'})
    selected=json.load(open(out/'selection.json'))['common_scale_hz']
    def save(fig,name):
        fig.savefig(dest/(name+'.png'),dpi=160,bbox_inches='tight')
        fig.savefig(dest/(name+'.svg'),bbox_inches='tight');plt.close(fig)
    fig,ax=plt.subplots(3,2,figsize=(14,10),layout='constrained')
    envelopes={}
    for col,seg in enumerate(cfg['segments']):
        d=load_data(cfg,seg);t=d['t'];y=d['y'];rel=t-t[0];dt=np.diff(t);tm=(t[1:]+t[:-1])/2
        m=ContinuousPhase.load(out/f'{seg}_phase_model.json')
        hall=np.unwrap(d['w'].phase_rad.to_numpy());steps=np.diff(hall)-np.diff(y);idx=np.argmax(abs(steps))+1;center=t[idx]
        q=np.linspace(center-.12,center+.12,600);use=(t>q[0])&(t<q[-1]);im=(tm>q[0])&(tm<q[-1])
        ax[0,col].plot((t[use]-center)*1000,np.rad2deg(hall[use]-hall[idx-1]),'.-',color='#cc503e',label='旧相位（已 unwrap）')
        ax[0,col].plot((t[use]-center)*1000,np.rad2deg(y[use]-y[idx-1]),'.',color='#1e7490',label='编码器相位')
        ax[0,col].plot((q-center)*1000,np.rad2deg(m.query(q)-m.query(t[idx-1])),color='#20854d',label='新连续相位')
        ax[0,col].set_title(f'{seg}：最大归零修正 {np.rad2deg(steps[idx-1]):+.2f}°；HRT {center:.6f} s')
        ax[0,col].set_ylabel('相位变化（°）\n图示在复位前一个样本对齐')
        ax[1,col].plot((tm[im]-center)*1000,(np.diff(hall)/(2*np.pi*dt))[im],'.-',color='#cc503e',label='旧相位差分')
        ax[1,col].plot((tm[im]-center)*1000,(np.diff(y)/(2*np.pi*dt))[im],'.',color='#1e7490',label='编码器区间平均频率')
        ax[1,col].plot((q-center)*1000,m.query(q,1)/(2*np.pi),color='#20854d',label='新相位瞬时频率')
        ax[1,col].set_ylabel('频率（Hz）')
        ta=(tm[1:]+tm[:-1])/2;ia=(ta>q[0])&(ta<q[-1])
        olda=np.diff(np.diff(hall)/dt)/np.diff(tm);enca=np.diff(np.diff(y)/dt)/np.diff(tm)
        ax[2,col].plot((ta[ia]-center)*1000,olda[ia],'.-',color='#cc503e',label='旧相位二阶差分')
        ax[2,col].plot((ta[ia]-center)*1000,enca[ia],'.',color='#1e7490',label='编码器二阶差分')
        ax[2,col].plot((q-center)*1000,m.query(q,2),color='#20854d',label='新相位二阶导数')
        ax[2,col].set_ylabel('相位加速度（rad/s²）');ax[2,col].set_xlabel('相对观察到复位的时刻（ms）')
        for row in range(3):
            ax[row,col].grid(alpha=.2);ax[row,col].axvspan((t[idx-1]-center)*1000,0,color='#cc503e',alpha=.1)
            ax[row,col].legend(fontsize=8)
        grid=np.linspace(t[0],t[-1],5000);models=[ContinuousPhase.load(out/'fits'/f'{seg}_scale_{s}.json') for s in cfg['cutoff_scale_hz']]
        env={'t':grid,**m.flags(grid)}
        for k,name in enumerate(('phase','velocity','acceleration')):
            values=np.array([v.query(grid,k) for v in models]);env[name+'_min']=values.min(axis=0);env[name+'_max']=values.max(axis=0)
        pd.DataFrame(env).to_csv(out/f'{seg}_smoothing_envelope.csv',index=False)
        envelopes[seg]='min/max over frozen 8/16/32 scales, NOT confidence interval'
    fig.suptitle('先看红线与绿线：归零修正不再被解释成真实加减速\n上：连续相位；中：一阶导数；下：二阶导数。新轨迹只由编码器运动决定。',fontsize=15)
    save(fig,'01_reset_comparison')

    fig,ax=plt.subplots(3,2,figsize=(14,10),layout='constrained')
    for col,seg in enumerate(cfg['segments']):
        d=load_data(cfg,seg);t=d['t'];y=d['y'];m=ContinuousPhase.load(out/f'{seg}_phase_model.json');dt=np.diff(t);tm=(t[:-1]+t[1:])/2
        ax[0,col].plot(tm-t[0],np.diff(y)/(2*np.pi*dt),lw=.7,color='#1e7490',label='编码器区间平均频率')
        ax[0,col].plot(t-t[0],m.query(t,1)/(2*np.pi),lw=.7,color='#20854d',label='新轨迹瞬时频率')
        ax[0,col].plot(t-t[0],d['freq'].frequency_hz,lw=.8,color='#af8615',label='PX4 已低通频率')
        ax[0,col].set_title(f'{seg}：频率确有周期内变化，没有被压成常数')
        ax[0,col].set_ylabel('频率（Hz）');ax[0,col].legend(fontsize=8)
        core=(t>=d['core'][0])&(t<=d['core'][1]);dv=abs(np.diff(np.diff(y)/dt));idx=np.argmax(dv)+1;center=t[idx]
        q=np.linspace(max(center-.3,t[0]),min(center+.3,t[-1]),600)
        im=(tm>=q[0])&(tm<=q[-1]);it=(t>=q[0])&(t<=q[-1])
        ax[1,col].plot((tm[im]-center)*1000,(np.diff(y)/(2*np.pi*dt))[im],'.',color='#1e7490',label='编码器区间平均')
        for scale,color in zip(cfg['cutoff_scale_hz'],['#8b65a8','#d58f2d','#20854d']):
            mod=ContinuousPhase.load(out/'fits'/f'{seg}_scale_{scale}.json')
            ax[1,col].plot((q-center)*1000,mod.query(q,1)/(2*np.pi),color=color,label=f'样条尺度 {scale} Hz')
        ax[1,col].set_ylabel('频率（Hz）');ax[1,col].set_xlabel('相对编码器速度变化最大处（ms）');ax[1,col].legend(fontsize=8)
        for scale,color in zip(cfg['cutoff_scale_hz'],['#8b65a8','#d58f2d','#20854d']):
            mod=ContinuousPhase.load(out/'fits'/f'{seg}_scale_{scale}.json')
            ax[2,col].plot((q-center)*1000,mod.query(q,2),color=color,label=f'样条尺度 {scale} Hz')
        ax[2,col].set_ylabel('相位加速度（rad/s²）');ax[2,col].set_xlabel('相对编码器速度变化最大处（ms）');ax[2,col].legend(fontsize=8)
        for row in range(3):ax[row,col].grid(alpha=.2)
    fig.suptitle('参数敏感性：弱平滑保留编码器变化，但二阶导数仍依赖平滑选择\n三档均预先固定；32 Hz 由计数稀疏留出选择，未按 IMU 效果选参。尺度不是精确截止频率。',fontsize=14)
    save(fig,'02_frequency_and_sensitivity')

    fig,ax=plt.subplots(2,2,figsize=(13,8),layout='constrained')
    metrics=pd.read_csv(out/'observation_metrics.csv')
    for col,seg in enumerate(cfg['segments']):
        for row,(split,title) in enumerate([('sparse_selection','稀疏点留出：用于选参'),('block_validation','四个 60 ms 区间留出：选参后验证')]):
            z=metrics[(metrics.segment==seg)&(metrics.split==split)]
            ax[row,col].bar(z.scale_hz.astype(str),z.held_rmse_count,color=['#8b65a8','#d58f2d','#20854d','#8d969e'])
            for i,v in enumerate(z.held_rmse_count):ax[row,col].text(i,v+2,f'{v:.1f}',ha='center')
            ax[row,col].set_title(seg+' — '+title);ax[row,col].set_ylabel('留出 RMSE（编码器计数）')
            ax[row,col].set_xlabel('样条尺度（Hz）；linear = 线性插值');ax[row,col].grid(axis='y',alpha=.2)
    fig.suptitle('相位预测误差：连续缺口比单点缺失更难，不能当成同等验证\n1 个计数约 0.0111°；这些是同源开发数据留出，不是独立物理真值。',fontsize=14)
    save(fig,'03_count_holdouts')

    fig,ax=plt.subplots(2,2,figsize=(13,8),layout='constrained')
    z=pd.read_csv(out/'cycle_metrics.csv')
    for col,seg in enumerate(cfg['segments']):
        for row,sensor in enumerate(('accel','gyro')):
            for j,phase in enumerate(('hall_indexed','continuous_encoder')):
                v=z[(z.segment==seg)&(z.sensor==sensor)&(z.phase_model==phase)]
                ax[row,col].bar(np.arange(2)+.32*j,v.rmse_vector,width=.3,label='旧 Hall 相位' if j==0 else '新编码器连续相位',color=['#cc503e','#20854d'][j])
                for i,value in enumerate(v.rmse_vector):ax[row,col].text(i+.32*j,value,f'{value:.3f}',ha='center',va='bottom',fontsize=10)
            ax[row,col].set_xticks(np.arange(2)+.16,['分散完整翼拍留出','末尾八翼拍留出'])
            ax[row,col].set_title(f'{seg} — '+('加速度计比力' if sensor=='accel' else '陀螺仪'))
            ax[row,col].set_ylabel('向量 RMSE（'+('m/s²' if sensor=='accel' else 'rad/s')+'）');ax[row,col].legend(fontsize=9)
            ax[row,col].set_ylim(0,ax[row,col].get_ylim()[1]*1.13)
    fig.suptitle('相同完整翼拍、相同三阶回归：新相位总体小幅改善，并非全部指标都改善\nB 末尾八翼拍陀螺仪略变差；这不证明机体刚体运动已经更准确。',fontsize=14)
    save(fig,'04_cycle_prediction')

    fig,ax=plt.subplots(2,2,figsize=(13,8),layout='constrained')
    harmonics=pd.read_csv(out/'frequency_harmonics.csv')
    for col,seg in enumerate(cfg['segments']):
        z=pd.read_csv(out/f'{seg}_frequency_spectrum.csv')
        for name,color,label in [('encoder_interval','#1e7490','编码器'),('spline_32_interval','#20854d','所选样条'),('spline_8_interval','#8b65a8','强平滑')]:
            ax[0,col].semilogy(z.frequency_hz,np.maximum(z[name],1e-12),color=color,label=label)
        ax[0,col].set(xlabel='频率（Hz）',ylabel='Lomb–Scargle 幅值平方尺度（Hz²）',title=f'{seg}：原生非均匀时刻；去线性趋势')
        ax[0,col].legend()
        for j,(name,color) in enumerate([('encoder_interval','#1e7490'),('spline_32_interval','#20854d'),('spline_8_interval','#8b65a8')]):
            h=harmonics[(harmonics.segment==seg)&(harmonics.signal==name)]
            ax[1,col].bar(np.arange(3)+.24*j,h.amplitude_hz,width=.23,color=color,label=name)
        ax[1,col].set_xticks(np.arange(3)+.24,['翼拍一阶','二阶','三阶']);ax[1,col].set_ylabel('周期频率变化幅值（Hz）');ax[1,col].legend(fontsize=8)
    fig.suptitle('保留的是编码器支持的转速变化；谱和谐波不能证明传感器误差为零',fontsize=14)
    save(fig,'05_frequency_content')

    fig,ax=plt.subplots(3,2,figsize=(13,9),layout='constrained')
    for col,seg in enumerate(cfg['segments']):
        d=load_data(cfg,seg);t=d['t'];y=d['y'];dt=np.diff(t);tm=(t[:-1]+t[1:])/2
        hall=np.unwrap(d['w'].phase_rad.to_numpy());ids=np.flatnonzero(np.diff(d['w'].phase_unwrapped_rad)<-np.pi)+1
        corrections=np.diff(hall)-np.diff(y)
        idx=ids[np.argmin(abs(abs(corrections[ids-1])-np.median(abs(corrections[ids-1]))))]
        center=t[idx];m=ContinuousPhase.load(out/f'{seg}_phase_model.json');q=np.linspace(center-.16,center+.16,600)
        it=(t>=q[0])&(t<=q[-1]);im=(tm>=q[0])&(tm<=q[-1])
        ax[0,col].plot((t[it]-center)*1000,np.rad2deg(d['w'].phase_rad.to_numpy()[it]),'.-',color='#cc503e')
        ax[0,col].set_title(f'{seg} 普通复位：额外修正 {np.rad2deg(corrections[idx-1]):+.3f}°')
        ax[0,col].set_ylabel('原始包装相位（°）\n360° 回到 0° 是表示方式')
        ax[1,col].plot((t[it]-center)*1000,np.rad2deg(hall[it]-hall[idx-1]),'.-',color='#cc503e',label='旧相位 unwrap')
        ax[1,col].plot((q-center)*1000,np.rad2deg(m.query(q)-m.query(t[idx-1])),color='#20854d',label='新连续相位')
        ax[1,col].set_ylabel('相位变化（°）\n图示在复位前对齐');ax[1,col].legend(fontsize=9)
        ax[2,col].plot((tm[im]-center)*1000,(np.diff(hall)/(2*np.pi*dt))[im],'.-',color='#cc503e',label='旧相位差分')
        ax[2,col].plot((tm[im]-center)*1000,(np.diff(y)/(2*np.pi*dt))[im],'.',color='#1e7490',label='编码器区间频率')
        ax[2,col].plot((q-center)*1000,m.query(q,1)/(2*np.pi),color='#20854d',label='新轨迹频率')
        ax[2,col].set_ylabel('频率（Hz）');ax[2,col].set_xlabel('相对普通复位时刻（ms）');ax[2,col].legend(fontsize=9)
        for row in range(3):ax[row,col].grid(alpha=.2)
    fig.suptitle('普通回绕与小幅 Hall 修正：大部分时段新旧相位几乎重合\n正常 360° 回绕可 unwrap；小幅额外修正仍不必变成运动导数。',fontsize=14)
    save(fig,'06_normal_wrap')

    files=sorted(dest.glob('*.png'))
    content=''.join('<figure><img alt="'+p.stem+'" src="data:image/png;base64,'+base64.b64encode(p.read_bytes()).decode()+'"></figure>' for p in files)
    (out/'index.html').write_text('''<!doctype html><html lang="zh-CN"><meta charset="utf-8"><title>连续翼相位重建</title>
<style>body{max-width:1400px;margin:30px auto;padding:0 20px;font:18px/1.7 sans-serif;color:#24343b;background:#fafafa}img{width:100%}figure{margin:24px 0;background:white;padding:10px;border:1px solid #ddd}</style>
<h1>Stage 02.3-A：连续机械周期相位</h1>
<p><b>结论：CONDITIONAL。</b>编码器能提供一条 C²、严格递增、统一求导的连续相位曲线。Hall 只固定一个参考零点，后续归零台阶不进入运动导数。
仍未校准绝对机械零点与采样延迟；二阶导数对时间误差和平滑选择敏感，不能视为真实机翼关节角加速度。</p>
<p><b>阅读顺序：</b>第一张看红线的归零尖峰与绿线；第二张确认绿线仍保留转速起伏；第三张看被留出计数的预测；第四张比较同一翼拍划分的 IMU 周期预测；第五张看频率内容。</p>
<p>仅复用 A/B 各 20 秒，全部结果是开发验证。没有六自由度联合优化、训练标签或网络训练。</p>'''+content+'</html>')
    dump(out/'plot_manifest.json',dict(script_sha=sha(__file__),figures=[str(p) for p in files],envelopes=envelopes))


if __name__=='__main__':main()
