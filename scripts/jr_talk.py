#!/usr/bin/env python3
# encoding: utf-8
# M9 v1: Chinese command -> robot task. Two parser backends:
#   rule  - offline keyword parser (default; covers the demo without any network)
#   llm   - Claude API structured-output parser (auto-enabled when ANTHROPIC_API_KEY
#           is set; understands arbitrary phrasing and emits detector-friendly
#           English prompts for the open-vocabulary detector -- the D3 vision)
#
#   python3 jr_talk.py "把红色方块都收好"           # parse + execute
#   python3 jr_talk.py --dry "帮我把牙刷拿过来"     # parse only, print the plan
#
# Actions: fetch (grab target, keep in gripper) / clear (grasp-all to paper)
#          / survey (look and report) / none (not a robot command)
import json
import os
import subprocess
import sys

GRASP = os.path.expanduser('~/jetrover_ws/jr_grasp_all.py')

# Chinese noun -> open-vocab English prompt (detector-friendly wording matters:
# "toy block" works, "cube" does not -- validated 2026-07-05)
NOUNS = {
    '方块': 'toy block', '积木': 'toy block', '牙刷': 'toothbrush', '笔': 'pen',
    'u盘': 'usb flash drive', 'U盘': 'usb flash drive', '优盘': 'usb flash drive',
    '香蕉': 'banana', '瓶': 'bottle', '瓶子': 'bottle', '可乐': 'bottle',
    '遥控': 'remote control', '遥控器': 'remote control', '勺': 'spoon', '勺子': 'spoon',
    '羽毛球': 'badminton shuttlecock', '球': 'ball', '苹果': 'apple', '橘子': 'orange',
    '杯子': 'cup', '杯': 'cup', '剪刀': 'scissors', '手机': 'cell phone', '钥匙': 'key',
}
COLORS = {'红': 'red', '绿': 'green', '蓝': 'blue', '黄': 'yellow',
          '橙': 'orange', '紫': 'purple', '青': 'cyan'}


def parse_rule(text):
    # offline keyword parser: verb -> action, nouns/colors -> targets
    t = text.strip()
    targets, colors, zh_words = [], [], []
    for zh, en in NOUNS.items():
        if zh in t and en not in targets:
            targets.append(en); zh_words.append(zh)
    for zh, en in COLORS.items():
        if zh in t and en not in colors:
            colors.append(en); zh_words.insert(0, zh + '色')
    if any(w in t for w in ('看', '扫', '有什么', '有哪些', '检查')) and \
       not any(w in t for w in ('抓', '拿', '取', '捡', '收')):
        action = 'survey'
    elif any(w in t for w in ('拿来', '拿过来', '取来', '拿给', '递给', '带来')):
        action = 'fetch'
    elif any(w in t for w in ('抓', '拿', '取', '捡', '收', '清')):
        action = 'clear' if any(w in t for w in ('都', '全', '所有', '清')) else 'fetch'
    else:
        return {'action': 'none', 'targets_en': [],
                'reply': '没听懂。试试:"把红色方块都收好" / "把牙刷拿过来" / "看看地上有什么"'}
    reply = '好,%s%s' % ({'fetch': '去拿', 'clear': '收拾', 'survey': '看看'}[action],
                          '、'.join(zh_words) or '地上的东西')
    return {'action': action, 'targets_en': targets, 'colors': colors, 'reply': reply}


def parse_llm(text):
    # Claude structured-output parser -- understands arbitrary phrasing and produces
    # detector-friendly English prompts. Model per env (default claude-opus-4-8).
    import anthropic
    client = anthropic.Anthropic()
    schema = {
        'type': 'object',
        'properties': {
            'action': {'type': 'string', 'enum': ['fetch', 'clear', 'survey', 'none']},
            'targets_en': {'type': 'array', 'items': {'type': 'string'}},
            'reply': {'type': 'string'},
        },
        'required': ['action', 'targets_en', 'reply'],
        'additionalProperties': False,
    }
    system = (
        '你是 JetRover 移动抓取机器人的指令解析器。把用户的中文指令解析成任务:\n'
        'action: fetch=抓一个目标并拿住 / clear=把目标全部抓起放到白纸收纳区 / '
        'survey=只观察汇报不动手 / none=与机器人任务无关。\n'
        'targets_en: 给开放词表检测器(YOLO-World)的英文提示词列表,小写常用名词,'
        '例如 "toy block"(不要用 cube)、"usb flash drive"、"toothbrush";'
        '颜色词直接并入提示词如 "red toy block";用户没限定目标就给空列表。\n'
        'reply: 给用户的一句简短中文确认。\n'
        '约束:夹爪张口 48mm、只抓轻的小物体;听不懂或超能力范围就 action=none 并在 reply 里说明。'
    )
    resp = client.messages.create(
        model=os.environ.get('JR_LLM_MODEL', 'claude-opus-4-8'),
        max_tokens=1000,
        system=system,
        output_config={'format': {'type': 'json_schema', 'schema': schema}},
        messages=[{'role': 'user', 'content': text}],
    )
    out = next(b.text for b in resp.content if b.type == 'text')
    return json.loads(out)


def execute(plan):
    action = plan['action']
    if action == 'none':
        print(plan.get('reply', '')); return 0
    env = dict(os.environ)
    env.setdefault('JR_DUR', '2.0')
    targets = plan.get('targets_en', [])
    if targets:
        env['JR_YOLO'] = '1'
        env['JR_YOLO_CLASSES'] = ','.join(targets)
        env['JR_TARGET'] = ','.join(t.replace(' ', '_') for t in targets)
    if plan.get('colors') and not targets:
        env['JR_TARGET'] = ','.join(plan['colors'])   # color labels need no YOLO
    if action == 'fetch':
        env['JR_CARRY'] = '1'
    mode = 'survey' if action == 'survey' else 'run'
    print(plan.get('reply', ''))
    print('[TALK] exec: %s %s (JR_TARGET=%s JR_YOLO=%s)' %
          (GRASP, mode, env.get('JR_TARGET', '-'), env.get('JR_YOLO', '0')))
    return subprocess.call(['python3', GRASP, mode], env=env)


def main():
    args = [a for a in sys.argv[1:] if a != '--dry']
    dry = '--dry' in sys.argv
    if not args:
        print('usage: jr_talk.py [--dry] "中文指令"'); return
    text = ' '.join(args)
    use_llm = os.environ.get('ANTHROPIC_API_KEY') and os.environ.get('JR_LLM', '1') != '0'
    if use_llm:
        try:
            plan = parse_llm(text)
            plan['backend'] = 'llm'
        except Exception as e:
            print('[TALK] llm parse failed (%s), falling back to rules' % e)
            plan = parse_rule(text); plan['backend'] = 'rule'
    else:
        plan = parse_rule(text); plan['backend'] = 'rule'
    print('[TALK] plan(%s): %s' % (plan['backend'], json.dumps(plan, ensure_ascii=False)))
    if not dry:
        sys.exit(execute(plan))


if __name__ == '__main__':
    main()
