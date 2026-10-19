#!/usr/bin/env python3
"""Plot measured phase resets against same-source encoder increments, without correcting data."""
import argparse
import base64
import hashlib
import json
import subprocess
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
import numpy as np
import pandas as pd


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--source', default='runs/stage02_2_20261017')
    ap.add_argument('--output', default='runs/phase_jump_20261019')
    args = ap.parse_args()
    source, out = Path(args.source), Path(args.output)
    out.mkdir(parents=True, exist_ok=False)
    font_path = subprocess.check_output(['fc-match', '-f', '%{file}', 'Noto Sans CJK SC'], text=True)
    font_manager.fontManager.addfont(font_path)
    plt.rcParams.update({'font.family': font_manager.FontProperties(fname=font_path).get_name(),
                         'axes.unicode_minus': False, 'font.size': 10, 'svg.fonttype': 'path'})
    inputs, all_data, summary, events = {}, {}, {}, []
    figures = []

    def save(fig, name):
        fig.savefig(out / (name + '.png'), dpi=165, bbox_inches='tight')
        fig.savefig(out / (name + '.svg'), bbox_inches='tight')
        plt.close(fig)
        figures.append(name)

    for seg in ('A', 'B'):
        folder = source / 'phase_raw' / seg
        meta = json.loads((folder / 'metadata.json').read_text())
        inputs[str(folder / 'metadata.json')] = sha(folder / 'metadata.json')
        for name in ('WingPhase.cpp', 'AS5600.cpp'):
            ref = source / 'reference' / 'px4' / f'{seg}_{name}'
            inputs[str(ref)] = sha(ref)
        frames = {}
        for filename in ('wing_phase_0.csv', 'encoder_count_0.csv', 'flap_frequency_0.csv'):
            path = folder / filename
            inputs[str(path)] = sha(path)
            assert inputs[str(path)] == meta['topics'][filename]['sha256']
            frames[filename] = pd.read_csv(path, float_precision='round_trip')
        w, enc = frames['wing_phase_0.csv'], frames['encoder_count_0.csv']
        t = w.timestamp.to_numpy() * 1e-6
        dt = np.diff(t)
        assert np.all(w.phase_valid == 1) and np.all((dt > 0) & (dt < .03))
        assert np.array_equal(w.timestamp, enc.timestamp)
        assert np.array_equal(w.encoder_total_count, enc.total_count)
        # All samples valid/contiguous in these two frozen contexts. Reject rather
        # than silently interpolate if another input violates this prerequisite.
        wrapped = w.phase_rad.to_numpy()
        phase = np.unwrap(wrapped)
        encoder_phase = phase[0] + (enc.total_count.to_numpy() - enc.total_count.iloc[0]) * (
            2 * np.pi / (4096 * meta['parameters']['FLAP_RATIO']))
        correction = np.diff(phase) - np.diff(encoder_phase)
        reset = np.diff(w.phase_unwrapped_rad.to_numpy()) < -np.pi
        ids = np.flatnonzero(reset) + 1
        assert np.max(np.abs(correction[~reset])) < 2e-6
        assert len(ids) == {'A': 78, 'B': 79}[seg]
        offset = np.rad2deg(phase - encoder_phase)
        freq_p = np.diff(phase) / (2 * np.pi * dt)
        freq_e = np.diff(encoder_phase) / (2 * np.pi * dt)
        tm = (t[1:] + t[:-1]) / 2
        t2 = (tm[1:] + tm[:-1]) / 2
        a_p = np.diff(2 * np.pi * freq_p) / np.diff(tm)
        a_e = np.diff(2 * np.pi * freq_e) / np.diff(tm)
        jump = np.rad2deg(correction[ids - 1])
        # 5 deg only separates the visibly disjoint clusters for description;
        # it is not a proposed phase rejection or smoothing threshold.
        large = np.abs(jump) > 5
        large_ids, large_jumps = ids[large], jump[large]
        assert len(large_ids) % 2 == 0
        paired = np.all(np.sign(large_jumps[::2]) == -np.sign(large_jumps[1::2]))
        stats = {
            'rows': len(t), 'duration_s': float(t[-1] - t[0]),
            'start_hrt_s': float(t[0]), 'end_hrt_s': float(t[-1]),
            'source_ulog_sha256': meta['source_sha256'], 'firmware': meta['firmware'],
            'resets': len(ids), 'positive_corrections': int(sum(jump > 0)),
            'negative_corrections': int(sum(jump < 0)),
            'abs_correction_deg_p50_p90_p95_max': np.quantile(abs(jump), [.5, .9, .95, 1]).tolist(),
            'small_cluster_count': int(sum(~large)), 'large_cluster_count': int(sum(large)),
            'small_cluster_abs_max_deg': float(max(abs(jump[~large]))),
            'large_cluster_abs_min_max_deg': [float(min(abs(jump[large]))), float(max(abs(jump[large])))],
            'nonreset_max_error_rad': float(max(abs(correction[~reset]))),
            'phase_frequency_hz_min_max': [float(min(freq_p)), float(max(freq_p))],
            'encoder_frequency_hz_min_max': [float(min(freq_e)), float(max(freq_e))],
            'max_abs_frequency_correction_hz': float(max(abs(freq_p - freq_e))),
            'max_abs_second_difference_correction_rad_s2': float(max(abs(a_p - a_e))),
            'offset_deg_min_max_end': [float(min(offset)), float(max(offset)), float(offset[-1])],
            'large_events_adjacent_in_large_event_list_opposite_sign': bool(paired),
            'large_event_pair_separation_s': (t[large_ids[1::2]] - t[large_ids[::2]]).tolist(),
            'large_event_pair_sum_deg': (large_jumps[::2] + large_jumps[1::2]).tolist(),
        }
        for k, idx in enumerate(ids):
            events.append(dict(segment=seg, reset_ordinal=k, interval_start_hrt_s=t[idx-1],
                               interval_end_hrt_s=t[idx], correction_deg=jump[k],
                               frequency_error_hz=freq_p[idx-1]-freq_e[idx-1],
                               previous_phase_deg=np.rad2deg(wrapped[idx-1]),
                               new_phase_deg=np.rad2deg(wrapped[idx]),
                               offset_after_deg=offset[idx], large_descriptive=bool(large[k])))
        samples = pd.DataFrame(dict(t_hrt_s=t, phase_wrapped_rad=wrapped,
                                    phase_unwrapped_offline_rad=phase,
                                    encoder_relative_phase_rad=encoder_phase, offset_deg=offset))
        samples.to_csv(out / f'{seg}_samples.csv', index=False)
        pd.DataFrame(dict(midpoint_hrt_s=tm, phase_difference_hz=freq_p,
                          encoder_difference_hz=freq_e, correction_deg=np.rad2deg(correction),
                          reset=reset)).to_csv(out / f'{seg}_increments.csv', index=False)
        pd.DataFrame(dict(midpoint_hrt_s=t2, phase_second_difference_rad_s2=a_p,
                          encoder_second_difference_rad_s2=a_e)).to_csv(out / f'{seg}_second_differences.csv', index=False)
        all_data[seg] = dict(t=t, wrapped=wrapped, offset=offset, tm=tm, t2=t2, fp=freq_p,
                            fe=freq_e, ap=a_p, ae=a_e, logged=w.flap_frequency_hz.to_numpy(),
                            ids=ids, jump=jump)
        summary[seg] = stats

        eligible = np.flatnonzero((t[ids] > t[0]+.4) & (t[ids] < t[-1]-.4))
        median = eligible[np.argmin(abs(abs(jump[eligible]) - np.median(abs(jump))))]
        largest = int(np.argmax(abs(jump)))
        fig, axes = plt.subplots(4, 2, figsize=(13, 11), layout='constrained')
        for col, (pick, label) in enumerate(((median, '接近中位数的普通修正'), (largest, '最大绝对修正'))):
            idx = ids[pick]; center = t[idx]
            selection = (t >= center-.33) & (t <= center+.33)
            ii = (tm >= center-.33) & (tm <= center+.33)
            jj = (t2 >= center-.33) & (t2 <= center+.33)
            x = (t-center)*1000
            axes[0, col].plot(x[selection], np.rad2deg(wrapped[selection]), '.-', color='#536273')
            axes[0, col].set_title(f'{label}\nHRT {center:.6f} s；修正 {jump[pick]:+.3f}°')
            axes[0, col].set_ylabel('原始 phase_rad（°）')
            axes[1, col].plot(x[selection], offset[selection]-offset[idx-1], '.-', color='#c54530')
            axes[1, col].axhline(0, color='gray', lw=.7)
            axes[1, col].set_ylabel('去回绕相位 − 编码器参考（°）\n在复位前一点对齐')
            axes[2, col].plot((tm[ii]-center)*1000, freq_p[ii], '.-', label='去回绕相位差分', color='#c54530')
            axes[2, col].plot((tm[ii]-center)*1000, freq_e[ii], '.-', label='编码器计数差分', color='#247b96')
            axes[2, col].plot(x[selection], w.flap_frequency_hz[selection], '--', label='记录频率（已低通）', color='#af8615')
            axes[2, col].set_ylabel('频率（Hz）')
            axes[3, col].plot((t2[jj]-center)*1000, a_p[jj], '.-', color='#c54530', label='相位二阶差分')
            axes[3, col].plot((t2[jj]-center)*1000, a_e[jj], '.-', color='#247b96', label='编码器二阶差分')
            axes[3, col].set_ylabel('二阶差分（rad/s²）')
            axes[3, col].set_xlabel('相对首次记录到复位的样本（ms）')
            for row in range(4):
                axes[row, col].axvspan((t[idx-1]-center)*1000, 0, color='#c54530', alpha=.12)
                axes[row, col].grid(alpha=.2)
                axes[row, col].set_xlim(-330, 330)
            stats[label+'_event_hrt_s'] = float(center)
        axes[2, 0].legend(fontsize=8, loc='upper left')
        axes[3, 0].legend(fontsize=8, loc='upper left')
        fig.suptitle(f'{seg} 航段：360° 回绕之外的额外修正及其求导影响\n阴影是日志中可定位的复位区间，不是已测 Hall 脉冲时刻', fontsize=14)
        save(fig, f'{seg}_local_details')

    fig, axes = plt.subplots(3, 2, figsize=(14, 9), layout='constrained')
    for col, seg in enumerate('AB'):
        d = all_data[seg]; rel = d['t']-d['t'][0]
        axes[0, col].plot(rel, d['offset'], lw=1.1, color='#247b96')
        axes[0, col].set_title(f'{seg}：{summary[seg]["resets"]} 次复位，{summary[seg]["large_cluster_count"]} 次大修正')
        axes[0, col].set_ylabel('去回绕相位 − 编码器参考（°）')
        axes[1, col].stem(rel[d['ids']], d['jump'], basefmt=' ', linefmt='#c54530', markerfmt='.')
        axes[1, col].set_ylabel('每次复位额外修正（°）')
        axes[2, col].plot(d['tm']-d['t'][0], d['fp'], color='#c54530', lw=.65, label='去回绕相位差分')
        axes[2, col].plot(d['tm']-d['t'][0], d['fe'], color='#247b96', lw=.7, label='编码器计数差分')
        axes[2, col].plot(rel, d['logged'], color='#af8615', lw=.9, label='记录频率（已低通）')
        axes[2, col].set_ylabel('频率（Hz）')
        axes[2, col].set_xlabel(f'距片段开始时间（s）；起点 HRT {d["t"][0]:.6f} s')
        for row in range(3):
            axes[row, col].grid(alpha=.2)
            axes[row, col].axhline(0, color='gray', lw=.5)
        axes[2, col].legend(fontsize=8)
    fig.suptitle('A/B 各约 20 秒原生记录：额外修正表现为台阶，部分在后续翼拍中反向恢复\n编码器参考与 phase 同源，仅用于定位软件归零修正，不是独立相位真值', fontsize=14)
    save(fig, 'phase_reset_overview')

    fig, axes = plt.subplots(2, 2, figsize=(12, 8), layout='constrained')
    for seg, color in [('A', '#247b96'), ('B', '#c54530')]:
        d = all_data[seg]; j = d['jump']
        axes[0, 0].hist(j, bins=np.linspace(-13, 13, 53), histtype='step', lw=2, label=seg, color=color)
        axes[0, 1].hist(j[abs(j) <= 5], bins=np.linspace(-1, 1, 41), histtype='step', lw=2, label=seg, color=color)
        axes[1, 0].scatter(j[:-1], j[1:], s=20, alpha=.65, label=seg, color=color)
        sample = d['ids']-1
        axes[1, 1].scatter(j, (d['fp']-d['fe'])[sample], s=20, alpha=.65, label=seg, color=color)
    axes[0, 0].set(xlabel='复位额外修正（°）', ylabel='次数', title='全部 157 次：中心簇与 ±12° 簇')
    axes[0, 1].set(xlabel='复位额外修正（°）', ylabel='次数', title='中心簇放大（|修正| ≤ 5°）')
    axes[1, 0].set(xlabel='本次复位修正（°）', ylabel='下一次复位修正（°）', title='相邻复位关系；不等于相邻大修正')
    axes[1, 1].set(xlabel='复位额外修正（°）', ylabel='相位差分 − 编码器差分（Hz）', title='差分放大效应：δf = δφ / (2π Δt)')
    for ax in axes.flat:
        ax.legend(); ax.grid(alpha=.2)
    fig.suptitle('修正幅度与差分影响；5° 仅为描述分簇，不是处理阈值', fontsize=14)
    save(fig, 'reset_statistics')

    pd.DataFrame(events).to_csv(out / 'reset_events.csv', index=False)
    (out / 'summary.json').write_text(json.dumps(summary, indent=2, ensure_ascii=False)+'\n')
    table = ''.join(f'<tr><td>{s}</td><td>{v["resets"]}</td><td>{v["large_cluster_count"]}</td>'
                    f'<td>{v["abs_correction_deg_p50_p90_p95_max"][0]:.3f}°</td>'
                    f'<td>{v["abs_correction_deg_p50_p90_p95_max"][-1]:.3f}°</td></tr>' for s,v in summary.items())
    images = ''.join(f'<figure><img src="data:image/png;base64,{base64.b64encode((out/(name+".png")).read_bytes()).decode()}"'
                     f' alt="{name}"><figcaption>{name}</figcaption></figure>'
                     for name in ['phase_reset_overview', 'A_local_details', 'B_local_details', 'reset_statistics'])
    html = '''<!doctype html><html lang="zh-CN"><meta charset="utf-8"><title>翼相位跳变诊断</title>
<style>body{font:17px/1.7 sans-serif;max-width:1400px;margin:30px auto;padding:0 24px;color:#203040;background:#fafafa}
img{width:100%;height:auto}figure{background:white;margin:25px 0;padding:12px;border:1px solid #ddd}
table{border-collapse:collapse}td,th{padding:8px 18px;border:1px solid #ccc}code{background:#eee}</style>
<h1>翼相位跳变：先区分回绕与归零修正，再决定求导方法</h1>
<p>范围：Stage 02.2 已提取的 A/B 各 2000 条记录、约 20 秒；没有扩展为全九月统计，没有更改相位或重建结果。</p>
<p><b>已验证：</b>去掉正常 360° 回绕后，仍有少量约 ±12° 的额外修正；相位与编码器参考的差出现台阶。
在 10 ms 间隔下，这会制造约 3.3 Hz 的差分偏差，二次求导进一步放大。普通修正则小得多。</p>
<table><tr><th>航段</th><th>复位次数</th><th>|修正| &gt; 5°</th><th>|修正| 中位数</th><th>|修正| 最大值</th></tr>''' + table + '''</table>
<p><b>如何读图：</b>总览先看蓝色台阶；局部图从原始相位、去回绕残差、一阶差分到二阶差分逐层比较。
第二行用复位前一个样本对齐编码器参考，不是拟合新轨迹。阴影只能定位到日志采样区间，不代表独立 Hall 时间测量。</p>
<p><b>仍未知：</b>没有 Hall 事件记录，尚不能确认大修正来自脉冲触发、归零时间匹配、机构变化还是其他环节。
编码器参考也有量化与安装误差，不能把它直接当真值。</p>
<p><b>建议：</b>不要直接对原始或仅 unwrap 后的 phase 求导。优先研究以编码器增量约束相位演化、以 Hall/相位归零提供带不确定性的周期参考，
再得到统一的 C² 连续相位及一、二阶导数。不要把每次归零台阶强行拟合成真实加减速，也不要只删尖峰后任意插值。
这里提供诊断依据，尚未选定平滑强度或替换原相位。</p>''' + images + '</html>'
    (out / 'index.html').write_text(html)
    manifest = dict(command=f'python scripts/phase_jump_diagnostics.py --source {source} --output {out}',
                    git_head=subprocess.check_output(['git','rev-parse','HEAD'], text=True).strip(),
                    script_sha256=sha(__file__), inputs=inputs, source_ulog_sha256={s:summary[s]['source_ulog_sha256'] for s in summary},
                    packages={'numpy': np.__version__, 'pandas': pd.__version__, 'matplotlib': matplotlib.__version__},
                    method='Valid contiguous A/B context only; unwrap then compare increments to count/(4096*FLAP_RATIO). '
                           'Midpoint first and second divided differences, no smoothing, no phase correction applied.',
                    checks='Input CSV hashes match extraction metadata; timestamp/count equality; all valid/no gaps; '
                           'nonreset increment identity <2e-6 rad; 78/79 reset counts match prior audit.',
                    outputs={p.name:sha(p) for p in sorted(out.iterdir()) if p.is_file()})
    (out / 'manifest.json').write_text(json.dumps(manifest, indent=2, ensure_ascii=False)+'\n')
    print(json.dumps(summary, indent=2, ensure_ascii=False))


if __name__ == '__main__':
    main()
