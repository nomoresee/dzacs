#!/usr/bin/env python3
"""Analyze raw sigmoid class scores for accuracy evaluation."""
import pickle, numpy as np, glob, os

LABEL_FROM_ID = {
    0:"0",1:"1",2:"2",3:"3",4:"4",5:"5",6:"6",7:"7",8:"8",9:"9",
    10:"electrodrill",11:"headphones",12:"keyboard",
    13:"mobile_phone",14:"monitor",15:"mouse",
    16:"multimeter",17:"oscillograph",18:"pliers",
    19:"printer",20:"screwdriver",21:"soldering_iron",
    22:"speaker",23:"tape_measure",24:"wrench",
}
LABELS = [LABEL_FROM_ID[i] for i in range(25)]
ANCHOR0=[10,13,16,30,33,23]; ANCHOR1=[30,61,62,45,59,119]; ANCHOR2=[116,90,156,198,373,326]

def sigmoid(x):
    return 1.0/(1.0+np.exp(-np.clip(x,-500,500)))

def analyze(img_path, raw, conf_thresh=0.25):
    """Decode and return top-1 prediction."""
    g0,g1,g2 = 2704,676,169
    grids = [
        (0,g0,52,8,ANCHOR0),(g0,g0+g1,26,16,ANCHOR1),(g0+g1,g0+g1+g2,13,32,ANCHOR2)
    ]
    dets = []
    for (s,e,H,stride,anchors) in grids:
        cells = (e-s)//3
        for a in range(3):
            a_start = s + a*cells
            for c in range(cells):
                row = a_start + c
                if row >= raw.shape[0]: break
                v = raw[row]
                if v.ndim > 1: v = v.flatten()
                obj_conf = sigmoid(float(v[4]))
                if obj_conf < conf_thresh: continue
                gi = c//H; gj = c%H
                if gi >= H: continue
                cls_vals = np.array(v[5:30]).flatten()
                cls_probs = sigmoid(cls_vals)
                cid = int(np.argmax(cls_probs))
                score = float(cls_probs[cid] * obj_conf)
                if score < conf_thresh: continue
                bx = (sigmoid(float(v[0]))*2-0.5+gj)*stride
                by = (sigmoid(float(v[1]))*2-0.5+gi)*stride
                bw = (sigmoid(float(v[2]))*2)**2*anchors[a*2]
                bh = (sigmoid(float(v[3]))*2)**2*anchors[a*2+1]
                x1=bx-bw/2; y1=by-bh/2
                dets.append((cid, score, x1, y1, bw, bh))
    return dets

def iou(a, b):
    ax1,ay1,aw,ah = a[2],a[3],a[4],a[5]
    bx1,by1,bw,bh = b[2],b[3],b[4],b[5]
    ax2=ax1+aw; ay2=ay1+ah; bx2=bx1+bw; by2=by1+bh
    x1=max(ax1,bx1); y1=max(ay1,by1)
    x2=min(ax2,bx2); y2=min(ay2,by2)
    w=max(0.,x2-x1); h=max(0.,y2-y1)
    inter=w*h; union=aw*ah+bw*bh-inter+1e-9
    return inter/union

def nms_per_class(dets, iou_thresh=0.45):
    if not dets: return []
    from collections import defaultdict
    by_cls = defaultdict(list)
    for d in dets: by_cls[d[0]].append(d)
    result = []
    for cls_dets in by_cls.values():
        cls_dets.sort(key=lambda x:-x[1])
        kept = []
        for d in cls_dets:
            dominated = any(iou(p,d)>iou_thresh for p in kept)
            if not dominated: kept.append(d)
        result.extend(kept)
    return sorted(result, key=lambda x:-x[1])

def analyze_raw_only(img_path, raw, top_k=5):
    """Use max sigmoid class score per image (no decode needed)."""
    cls_max = sigmoid(np.array(raw[:, 5:30])).max(axis=0)
    # Also compute max over all proposals weighted by obj
    obj = sigmoid(np.array(raw[:, 4]))
    weighted_cls = sigmoid(np.array(raw[:, 5:30])) * obj.reshape(-1, 1)
    weighted_max = weighted_cls.max(axis=0)
    
    top_by_max = np.argsort(cls_max)[::-1][:top_k]
    top_by_weighted = np.argsort(weighted_max)[::-1][:top_k]
    return top_by_max, top_by_weighted

def main():
    # Load accuracy cache (has processed raw outputs)
    cache_path = '/tmp/rknn_acc_cache.pkl'
    if not os.path.exists(cache_path):
        print("Cache not found!")
        return
    
    cache = pickle.load(open(cache_path, 'rb'))
    print(f"Loaded {len(cache)} cached items")
    
    from collections import defaultdict
    stats_decode = defaultdict(lambda: {'total':0,'correct':0,'top5':0,'errors':[]})
    stats_raw_max = defaultdict(lambda: {'total':0,'correct':0,'errors':[]})
    stats_raw_weighted = defaultdict(lambda: {'total':0,'correct':0,'errors':[]})
    
    for img_path, result in cache.items():
        if 'raw' not in result: continue  # acc cache doesn't have raw
        expected = result.get('expected','?')
        if expected not in [LABEL_FROM_ID[i] for i in range(25)]: continue
        
        raw = result.get('raw')
        if raw is None: continue
        
        # Method 1: Raw max sigmoid class score (no decode)
        top_raw, top_weighted = analyze_raw_only(img_path, raw)
        
        expected_id = next((i for i,v in LABEL_FROM_ID.items() if v==expected), -1)
        
        # Raw max method
        stats_raw_max[expected]['total'] += 1
        if top_raw[0] == expected_id:
            stats_raw_max[expected]['correct'] += 1
        else:
            stats_raw_max[expected]['errors'].append((os.path.basename(img_path), LABEL_FROM_ID.get(top_raw[0],'?')))
        
        # Raw weighted method
        stats_raw_weighted[expected]['total'] += 1
        if top_weighted[0] == expected_id:
            stats_raw_weighted[expected]['correct'] += 1
        else:
            stats_raw_weighted[expected]['errors'].append((os.path.basename(img_path), LABEL_FROM_ID.get(top_weighted[0],'?')))
    
    # Print results
    print(f"\n{'='*70}")
    print(f"  Raw Max Sigmoid (no decode)")
    print(f"{'='*70}")
    total = sum(s['total'] for s in stats_raw_max.values())
    correct = sum(s['correct'] for s in stats_raw_max.values())
    print(f"  Overall: {correct}/{total} = {correct/total*100:.1f}%")
    for label in LABELS:
        if label in ('0','1','2','3','4','5','6','7','8','9'): continue
        s = stats_raw_max.get(label)
        if not s or s['total']==0: continue
        print(f"  {label:<20}: {s['correct']}/{s['total']} = {s['correct']/s['total']*100:.1f}%")
        if s['errors'][:3]: print(f"    Errors: {[(e[0],e[1]) for e in s['errors'][:3]]}")
    
    print(f"\n{'='*70}")
    print(f"  Raw Weighted Max (no decode)")
    print(f"{'='*70}")
    total = sum(s['total'] for s in stats_raw_weighted.values())
    correct = sum(s['correct'] for s in stats_raw_weighted.values())
    print(f"  Overall: {correct}/{total} = {correct/total*100:.1f}%")
    for label in LABELS:
        if label in ('0','1','2','3','4','5','6','7','8','9'): continue
        s = stats_raw_weighted.get(label)
        if not s or s['total']==0: continue
        print(f"  {label:<20}: {s['correct']}/{s['total']} = {s['correct']/s['total']*100:.1f}%")

if __name__ == '__main__':
    main()
