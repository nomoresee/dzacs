#!/usr/bin/env python3
"""Analyze fast_eval cache with correct label mapping."""
import pickle, numpy as np

cache_path = '/tmp/rknn_fast_eval.pkl'
results = pickle.load(open(cache_path, 'rb'))

LABEL_FROM_ID = {
    10: "electrodrill", 11: "headphones", 12: "keyboard",
    13: "mobile_phone", 14: "monitor", 15: "mouse",
    16: "multimeter", 17: "oscillograph", 18: "pliers",
    19: "printer", 20: "screwdriver", 21: "soldering_iron",
    22: "speaker", 23: "tape_measure", 24: "wrench",
}
LABEL_TO_ID = {v: k for k, v in LABEL_FROM_ID.items()}

stats = {k: {'total':0,'correct':0,'scores':[],'errors':[]} for k in LABEL_TO_ID}

for img_path, r in results.items():
    expected = r['expected']
    if expected not in LABEL_TO_ID:
        continue

    wm = r['weighted_max']  # (25,) - already max over all proposals
    top_cid = int(np.argmax(wm))
    top_score = float(wm[top_cid])
    pred = LABEL_FROM_ID.get(top_cid, f"?{top_cid}?")

    stats[expected]['total'] += 1
    stats[expected]['scores'].append(top_score)
    if pred == expected:
        stats[expected]['correct'] += 1
    else:
        stats[expected]['errors'].append((r['fname'], pred, top_score))

total_correct = sum(s['correct'] for s in stats.values())
total_count = sum(s['total'] for s in stats.values())

print(f"{'='*65}")
print(f"  RKNN Accuracy Test Results")
print(f"  Model: aug_enhanced_v5s_opt2.rknn")
print(f"  Images: {total_count}/{len(results)}")
print(f"  Method: Weighted Max (sigmoid(cls) * sigmoid(obj))")
print(f"{'='*65}")
print(f"  {'Class':<20} {'Total':>6} {'Correct':>7} {'Top1%':>7} {'AvgScore':>10}")
print(f"  {'-'*55}")

for label in sorted(LABEL_TO_ID.keys()):
    s = stats[label]
    if s['total'] == 0: continue
    acc = s['correct']/s['total']*100
    avg = np.mean(s['scores']) if s['scores'] else 0
    print(f"  {label:<20} {s['total']:>6} {s['correct']:>7} {acc:>6.1f}% {avg:>10.3f}")

print(f"  {'-'*55}")
overall = total_correct/total_count*100 if total_count else 0
print(f"  {'OVERALL':<20} {total_count:>6} {total_correct:>7} {overall:>6.1f}%")
print(f"{'='*65}")

errors = []
for label, s in stats.items():
    for fname, pred, score in s['errors']:
        errors.append((label, fname, pred, score))
errors.sort(key=lambda x:-x[3])
print(f"\n  Most confident errors:")
for gt, fname, pred, score in errors[:15]:
    print(f"    {fname}: GT={gt} -> Pred={pred} ({score:.4f})")
