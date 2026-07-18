#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
RKNN Model Accuracy Test
Model: aug_enhanced_v5s_opt2.rknn (25-class YOLOv5s, float output)
Labels: model idx 10=electrodrill, 11=headphones, ..., 24=wrench
Input: 416x416 uint8 BGR (SDK auto-normalizes)
Output: (1, 10647, 30) single concatenated YOLO tensor
"""

import cv2
import numpy as np
from rknn.api import RKNN
from collections import defaultdict
import glob
import time
import os
import argparse
import pickle

# Correct label mapping: model class index -> name
# (matches postprocess_25.cc and confirmed by testing)
LABEL_FROM_ID = {
    10: "electrodrill", 11: "headphones", 12: "keyboard",
    13: "mobile_phone", 14: "monitor", 15: "mouse",
    16: "multimeter", 17: "oscillograph", 18: "pliers",
    19: "printer", 20: "screwdriver", 21: "soldering_iron",
    22: "speaker", 23: "tape_measure", 24: "wrench",
}
LABELS = [LABEL_FROM_ID[i] for i in range(25)]
LABEL_TO_ID = {v: k for k, v in LABEL_FROM_ID.items()}

MODEL_H = 416
MODEL_W = 416


def sigmoid(x):
    return 1.0 / (1.0 + np.exp(-np.clip(x, -500, 500)))


def preprocess(img):
    """Letterbox resize to 416x416."""
    orig_h, orig_w = img.shape[:2]
    scale = min(MODEL_W / orig_w, MODEL_H / orig_h)
    nw = int(orig_w * scale); nh = int(orig_h * scale)
    canvas = np.full((MODEL_H, MODEL_W, 3), 128, dtype=np.uint8)
    top = (MODEL_H - nh) // 2; left = (MODEL_W - nw) // 2
    canvas[top:top+nh, left:left+nw] = cv2.resize(img, (nw, nh))
    return canvas


def predict_top1(out):
    """
    Predict top-1 class using Weighted Max method:
    max over all 10647 proposals of (sigmoid(class) * sigmoid(obj))
    This matches the C++ postprocess behavior where the highest-scoring
    proposal determines the predicted class.
    Returns: (class_name, score)
    """
    if out.ndim == 3:
        out = out.reshape(-1, out.shape[-1])  # (10647, 30)

    # sigmoid(class) * sigmoid(obj) for all proposals
    obj = sigmoid(np.array(out[:, 4]))                    # (10647,)
    cls = sigmoid(np.array(out[:, 5:30]))                 # (10647, 25)
    weighted = cls * obj.reshape(-1, 1)                  # (10647, 25)
    weighted_max = weighted.max(axis=0)                   # (25,)

    top_cid = int(np.argmax(weighted_max))
    top_score = float(weighted_max[top_cid])
    label = LABEL_FROM_ID.get(top_cid, f"?{top_cid}?")
    return label, top_score, top_cid


def main():
    parser = argparse.ArgumentParser(description="RKNN Model Accuracy Test")
    parser.add_argument('--model', default='/home/duzhong/dzacs/src/rknn_pt/model/aug_enhanced_v5s_opt2.rknn')
    parser.add_argument('--data_dir', default='/home/duzhong/dzacs/dataset/视觉组数据集')
    parser.add_argument('--cache', default='/tmp/rknn_test_cache.pkl')
    parser.add_argument('--report_every', type=int, default=200)
    args = parser.parse_args()

    # Load cache
    results = {}
    if os.path.exists(args.cache):
        results = pickle.load(open(args.cache, 'rb'))
        print(f"[INFO] Loaded {len(results)} cached results from {args.cache}")

    # Init RKNN
    print(f"[INFO] Loading model: {args.model}")
    rknn = RKNN()
    ret = rknn.load_rknn(args.model)
    if ret != 0:
        print(f"[ERROR] load_rknn failed: {ret}"); return
    print("[INFO] Initializing RKNN runtime...")
    ret = rknn.init_runtime(target='RK3588')
    if ret != 0:
        print(f"[ERROR] init_runtime failed: {ret}"); return
    print("[INFO] RKNN ready. Starting inference...")

    # Find all images
    all_imgs = sorted(glob.glob(os.path.join(args.data_dir, '*', '*.jpg')))
    print(f"[INFO] Found {len(all_imgs)} images in {args.data_dir}")

    t0 = time.time()
    for idx, img_path in enumerate(all_imgs):
        if img_path in results:
            if (idx + 1) % args.report_every == 0:
                elapsed = time.time() - t0
                rate = (idx + 1) / elapsed
                print(f"  [{idx+1}/{len(all_imgs)}] {rate:.1f} img/s (cached)")
            continue

        expected = os.path.basename(img_path).rsplit('_', 1)[0]
        if expected not in LABEL_TO_ID:
            results[img_path] = {'expected': expected, 'pred': None, 'error': 'unknown class'}
            continue

        img = cv2.imread(img_path)
        if img is None:
            results[img_path] = {'expected': expected, 'pred': None, 'error': 'read failed'}
            continue

        canvas = preprocess(img)
        raw = rknn.inference(inputs=[canvas])[0]

        pred, score, cid = predict_top1(raw)
        results[img_path] = {
            'expected': expected,
            'expected_id': LABEL_TO_ID[expected],
            'pred': pred,
            'score': score,
            'pred_id': cid,
        }

        # Save cache periodically
        if (idx + 1) % 100 == 0:
            pickle.dump(results, open(args.cache, 'wb'))

        if (idx + 1) % args.report_every == 0:
            elapsed = time.time() - t0
            rate = (idx + 1) / elapsed
            eta = (len(all_imgs) - idx - 1) / rate
            print(f"  [{idx+1}/{len(all_imgs)}] {rate:.1f} img/s | ETA: {eta/60:.1f}min")

    # Save final cache
    pickle.dump(results, open(args.cache, 'wb'))
    total_time = time.time() - t0

    # Evaluate
    stats = defaultdict(lambda: {'total': 0, 'correct': 0, 'scores': [], 'errors': []})
    for img_path, r in results.items():
        expected = r.get('expected', '')
        if expected not in LABEL_TO_ID:
            continue
        pred = r.get('pred')
        score = r.get('score', 0.0)
        stats[expected]['total'] += 1
        stats[expected]['scores'].append(score)
        if pred == expected:
            stats[expected]['correct'] += 1
        else:
            stats[expected]['errors'].append((os.path.basename(img_path), pred or 'NONE', score))

    # Print results
    total_correct = sum(s['correct'] for s in stats.values())
    total_count = sum(s['total'] for s in stats.values())

    print(f"\n{'='*65}")
    print(f"  RKNN Accuracy Test Results")
    print(f"  Model: {os.path.basename(args.model)}")
    print(f"  Images: {total_count}/{len(all_imgs)}, Time: {total_time/60:.1f}min")
    print(f"  Method: Weighted Max (sigmoid(class) * sigmoid(obj))")
    print(f"{'='*65}")
    print(f"  {'Class':<20} {'Total':>6} {'Correct':>7} {'Top1%':>7} {'AvgScore':>10}")
    print(f"  {'-'*55}")

    for label in sorted(LABEL_TO_ID.keys()):
        s = stats.get(label, {'total': 0, 'correct': 0, 'scores': []})
        if s['total'] == 0:
            continue
        acc = s['correct'] / s['total'] * 100
        avg = np.mean(s['scores']) if s['scores'] else 0.0
        print(f"  {label:<20} {s['total']:>6} {s['correct']:>7} {acc:>6.1f}% {avg:>10.3f}")

    print(f"  {'-'*55}")
    overall = total_correct / total_count * 100 if total_count else 0
    print(f"  {'OVERALL':<20} {total_count:>6} {total_correct:>7} {overall:>6.1f}%")
    print(f"{'='*65}")

    # Top errors
    errors = []
    for label, s in stats.items():
        for fname, pred, score in s['errors']:
            errors.append((label, fname, pred, score))
    errors.sort(key=lambda x: -x[3])
    print(f"\n  Most confident errors (by score):")
    for gt, fname, pred, score in errors[:15]:
        print(f"    {fname}: GT={gt} → Pred={pred} ({score:.4f})")

    rknn.release()


if __name__ == '__main__':
    main()
