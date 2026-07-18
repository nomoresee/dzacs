#!/usr/bin/env python3
"""Fast evaluation using raw sigmoid class scores (no box decode)."""
import cv2, numpy as np
from rknn.api import RKNN
import glob, pickle, os, time

MODEL_W, MODEL_H = 416, 416

def sigmoid(x):
    return 1.0/(1.0+np.exp(-np.clip(x,-500,500)))

def preprocess(img):
    orig_h, orig_w = img.shape[:2]
    scale = min(MODEL_W/orig_w, MODEL_H/orig_h)
    nw,nh = int(orig_w*scale), int(orig_h*scale)
    canvas = np.full((MODEL_H, MODEL_W, 3), 128, dtype=np.uint8)
    top=(MODEL_H-nh)//2; left=(MODEL_W-nw)//2
    canvas[top:top+nh,left:left+nw] = cv2.resize(img,(nw,nh))
    return canvas

def main():
    cache_path = '/tmp/rknn_fast_eval.pkl'
    if os.path.exists(cache_path):
        results = pickle.load(open(cache_path,'rb'))
        print(f"Loaded {len(results)} cached results")
    else:
        results = {}

    rknn = RKNN()
    rknn.load_rknn('/home/duzhong/dzacs/src/rknn_pt/model/aug_enhanced_v5s_opt2.rknn')
    print("Init runtime...")
    rknn.init_runtime(target='RK3588')

    all_imgs = sorted(glob.glob('/home/duzhong/dzacs/dataset/视觉组数据集/*/*.jpg'))
    print(f"Total: {len(all_imgs)} images")

    t0 = time.time()
    for idx, img_path in enumerate(all_imgs):
        if img_path in results:
            if (idx+1)%500==0: print(f"  [{idx+1}/{len(all_imgs)}] cached")
            continue

        img = cv2.imread(img_path)
        if img is None: continue
        canvas = preprocess(img)
        raw = rknn.inference(inputs=[canvas])[0]
        if raw.ndim == 3: raw = raw.reshape(-1, raw.shape[-1])

        # Method 1: max sigmoid class score across all proposals (no decode)
        cls = sigmoid(np.array(raw[:, 5:30]))  # (10647, 25)
        cls_max = cls.max(axis=0)  # (25,)

        # Method 2: max sigmoid class * obj confidence
        obj = sigmoid(np.array(raw[:, 4]))  # (10647,)
        weighted = cls * obj.reshape(-1, 1)
        weighted_max = weighted.max(axis=0)  # (25,)

        results[img_path] = {
            'fname': os.path.basename(img_path),
            'expected': os.path.basename(img_path).rsplit('_', 1)[0],
            'cls_max': cls_max.copy(),
            'weighted_max': weighted_max.copy(),
        }

        if (idx+1)%100==0:
            elapsed = time.time()-t0
            rate = (idx+1)/elapsed
            eta = (len(all_imgs)-idx-1)/rate
            pickle.dump(results, open(cache_path,'wb'))
            print(f"  [{idx+1}/{len(all_imgs)}] {rate:.1f} img/s | ETA: {eta/60:.1f}min")

    pickle.dump(results, open(cache_path,'wb'))
    total_time = time.time()-t0
    print(f"Done! {len(results)} images in {total_time/60:.1f}min ({len(results)/total_time:.1f} img/s)")
    rknn.release()

    # Evaluate
    MAP = {
        10:"electrodrill", 11:"headphones", 12:"keyboard",
        13:"mobile_phone", 14:"monitor", 15:"mouse",
        16:"multimeter", 17:"oscillograph", 18:"pliers",
        19:"printer", 20:"screwdriver", 21:"soldering_iron",
        22:"speaker", 23:"tape_measure", 24:"wrench",
    }
    real_classes = set(MAP.values())

    from collections import defaultdict
    stats_raw = defaultdict(lambda:{'total':0,'correct':0,'errors':[]})
    stats_weighted = defaultdict(lambda:{'total':0,'correct':0,'errors':[]})

    for img_path, r in results.items():
        expected = r['expected']
        if expected not in real_classes: continue

        # Raw max method
        cls_max = r['cls_max']
        top_raw = sorted(enumerate(cls_max), key=lambda x:-x[1])[0][0]
        pred_raw = MAP.get(top_raw, f'?{top_raw}?')
        stats_raw[expected]['total'] += 1
        if pred_raw == expected:
            stats_raw[expected]['correct'] += 1
        else:
            stats_raw[expected]['errors'].append((r['fname'], pred_raw, cls_max[top_raw]))

        # Weighted max method
        wm = r['weighted_max']
        top_wm = sorted(enumerate(wm), key=lambda x:-x[1])[0][0]
        pred_wm = MAP.get(top_wm, f'?{top_wm}?')
        stats_weighted[expected]['total'] += 1
        if pred_wm == expected:
            stats_weighted[expected]['correct'] += 1
        else:
            stats_weighted[expected]['errors'].append((r['fname'], pred_wm, wm[top_wm]))

    print(f"\n{'='*65}")
    print(f"  Evaluation: raw max sigmoid class scores")
    print(f"  Images: {len(results)}, Time: {total_time/60:.1f}min")
    print(f"{'='*65}")
    print(f"  {'Class':<20} {'Total':>6} {'Top1':>7} {'Acc%':>7}")
    print(f"  {'-'*45}")

    for mname, stats in [("Raw Max", stats_raw), ("Weighted Max", stats_weighted)]:
        total_c = sum(s['correct'] for s in stats.values())
        total_n = sum(s['total'] for s in stats.values())
        print(f"\n  [{mname}]")
        for label in sorted(real_classes):
            s = stats.get(label, {'total':0,'correct':0})
            if s['total'] == 0: continue
            acc = s['correct']/s['total']*100
            print(f"    {label:<20} {s['total']:>5} {s['correct']:>5} {acc:>6.1f}%")
        if total_n:
            print(f"    {'OVERALL':<20} {total_n:>5} {total_c:>5} {total_c/total_n*100:>6.1f}%")

        # Show errors
        errors = []
        for label, s in stats.items():
            for fname, pred, score in s['errors']:
                errors.append((label, fname, pred, score))
        errors.sort(key=lambda x:-x[3])
        if errors:
            print(f"\n    Most confident errors:")
            for gt, fname, pred, score in errors[:10]:
                print(f"      {fname}: GT={gt} → Pred={pred} ({score:.4f})")

if __name__ == '__main__':
    main()
