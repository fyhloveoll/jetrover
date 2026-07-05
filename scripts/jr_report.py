#!/usr/bin/env python3
# encoding: utf-8
# M7 evaluation report: parse the cumulative grasp_stats.csv into a quantified
# engineering report (rates, breakdowns by angle/width/color, per-session).
#   python3 jr_report.py [csv_path]     # default ~/jetrover_ws/grasp_stats.csv
# Writes grasp_report.md next to the CSV and prints it.
import os
import sys
import time

CSV = sys.argv[1] if len(sys.argv) > 1 else os.path.expanduser('~/jetrover_ws/grasp_stats.csv')
GOOD = ('SUCCESS', 'CARRYING')                       # cube ended up controlled
GRASP_TRIES = GOOD + ('MISS', 'GRASP_MISS')          # arm physically attempted a grasp
INFRA = ('DRIVE_FETCH', 'DRIVE_STRAFE')              # base moves, not grasp attempts


def load(path):
    rows = []
    for i, line in enumerate(open(path)):
        if i == 0 or not line.strip():
            continue
        p = line.strip().split(',')
        if len(p) < 9:
            continue
        rows.append({'ts': float(p[0]), 'id': p[1], 'label': p[2], 'u': int(p[3]),
                     'v': int(p[4]), 'angle': float(p[5]), 'width': float(p[6]),
                     'result': p[7], 'msg': p[8]})
    return rows


def rate(rows):
    tries = [r for r in rows if r['result'] in GRASP_TRIES]
    good = [r for r in tries if r['result'] in GOOD]
    return len(good), len(tries), (100.0 * len(good) / len(tries)) if tries else 0.0


def line_for(name, subset):
    g, t, p = rate(subset)
    return '| %s | %d/%d | %.0f%% |' % (name, g, t, p) if t else None


def main():
    rows = load(CSV)
    if not rows:
        print('no data in %s' % CSV)
        return
    out = []
    out.append('# JetRover 抓取系统评估报告')
    out.append('')
    out.append('生成时间:%s   数据:%s(%d 条记录)' %
               (time.strftime('%Y-%m-%d %H:%M'), os.path.basename(CSV), len(rows)))
    out.append('')

    g, t, p = rate(rows)
    nresults = {}
    for r in rows:
        nresults[r['result']] = nresults.get(r['result'], 0) + 1
    out.append('## 总体')
    out.append('')
    out.append('- **抓取成功率:%d/%d = %.0f%%**(物理尝试口径;底盘移动等基础动作不计)' % (g, t, p))
    out.append('- 结果分布:' + ', '.join('%s=%d' % kv for kv in sorted(nresults.items())))
    out.append('')

    out.append('## 分维度')
    out.append('')
    out.append('| 维度 | 成功/尝试 | 成功率 |')
    out.append('|---|---|---|')
    tries = [r for r in rows if r['result'] in GRASP_TRIES]
    for name, lo, hi in (('角度 |0-10°|', 0, 10), ('角度 |10-25°|', 10, 25), ('角度 |25-45°|', 25, 45)):
        l = line_for(name, [r for r in tries if lo <= abs(r['angle']) <= hi])
        if l:
            out.append(l)
    for name, lo, hi in (('宽度 <30mm', 0, 30), ('宽度 30-40mm', 30, 40), ('宽度 >40mm', 40, 999)):
        l = line_for(name, [r for r in tries if lo <= r['width'] < hi])
        if l:
            out.append(l)
    for lab in sorted(set(r['label'] for r in tries)):
        l = line_for('颜色 %s' % lab, [r for r in tries if r['label'] == lab])
        if l:
            out.append(l)
    out.append('')

    # sessions split on >30min gaps
    out.append('## 分场次(间隔 >30min 记新场)')
    out.append('')
    out.append('| 场次 | 时间 | 成功/尝试 | 成功率 | 记录数 |')
    out.append('|---|---|---|---|---|')
    sess, cur, last = [], [], None
    for r in rows:
        if last is not None and r['ts'] - last > 1800:
            sess.append(cur); cur = []
        cur.append(r); last = r['ts']
    if cur:
        sess.append(cur)
    for i, srows in enumerate(sess):
        g2, t2, p2 = rate(srows)
        when = time.strftime('%m-%d %H:%M', time.localtime(srows[0]['ts']))
        out.append('| %d | %s | %d/%d | %.0f%% | %d |' % (i + 1, when, g2, t2, p2, len(srows)))
    out.append('')
    out.append('注:CSV 含开发调试轮(参数实验/板子锁死幻影轮),正式指标建议以"干净基准场"'
               '(清零 CSV 后按固定协议连跑)为准;历史干净配方净值 21/31=68%(2026-06-30)。')

    md = '\n'.join(out)
    rp = os.path.join(os.path.dirname(CSV) or '.', 'grasp_report.md')
    open(rp, 'w').write(md + '\n')
    print(md)
    print('\n(written to %s)' % rp)


if __name__ == '__main__':
    main()
