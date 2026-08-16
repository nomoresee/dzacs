#!/usr/bin/env python3
"""Batch inference - save raw outputs only, no postprocess. Very fast."""
import cv2, numpy as np
from rknn.api import RKNN
import glob, pickle, os, time, argparse

MODEL_H, MODEL_W = 416, 416
ANCHOR0=[10,13,16,30,33,23]; ANCHOR1=[30,61,62,45,59,119]; ANCHOR2=[116,90,156,198,373,326]

def sigmoid(x):
    return 1.0/(1.0+np.exp(-np.clip(x,-500,500)))

def preprocess(img):
    orig_h, orig_w = img.shape[:2]
    scale = min(MODEL_W/orig_w, MODEL_H/orig_h)
    nw=int(orig_w*scale); nh=int(orig_h*scale)
    canvas = np.full((MODEL_H,MODEL_W,3),128,dtype=np.uint8)
    top=(MODEL_H-nh)//2; left=(MODEL_W-nw)//2
    canvas[top:top+nh,left:left+nw] = cv2.resize(img,(nw,nh))
    return canvas, scale, top, left, orig_w, orig_h

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--model', default='/home/duzhong/dzacs/src/rknn_pt/model/aug_enhanced_v5s_opt2.rknn')
    parser.add_argument('--data_dir', default='/home/duzhong/dzacs/dataset/视觉组数据集')
    parser.add_argument('--output', default='/tmp/rknn_raw_cache.pkl')
    parser.add_argument('--resume', action='store_true')
    args = parser.parse_args()

    cache = {}
    if args.resume and os.path.exists(args.output):
        cache = pickle.load(open(args.output,'rb'))
        print(f"Resuming with {len(cache)} cached items")

    rknn = RKNN()
    rknn.load_rknn(args.model)
    print("Initializing RKNN runtime...")
    rknn.init_runtime(target='RK3588')
    print("RKNN ready")

    all_imgs = sorted(glob.glob(os.path.join(args.data_dir,'*','*.jpg')))
    print(f"Total images: {len(all_imgs)}")

    t0 = time.time()
    for idx, img_path in enumerate(all_imgs):
        if img_path in cache:
            if (idx+1)%500==0: print(f"  [{idx+1}/{len(all_imgs)}] cached")
            continue

        img = cv2.imread(img_path)
        if img is None: continue
        canvas, scale, pad_t, pad_l, orig_w, orig_h = preprocess(img)

        t1 = time.time()
        raw = rknn.inference(inputs=[canvas])[0]
        if raw.ndim == 3: raw = raw.reshape(-1, raw.shape[-1])
        infer_ms = (time.time()-t1)*1000

        cache[img_path] = {
            'scale': scale, 'pad_t': pad_t, 'pad_l': pad_l,
            'orig_w': orig_w, 'orig_h': orig_h,
            'raw': raw, 'infer_ms': infer_ms,
            'fname': os.path.basename(img_path),
        }

        if (idx+1)%100==0:
            elapsed = time.time()-t0
            rate = (idx+1)/elapsed
            eta = (len(all_imgs)-idx-1)/rate
            pickle.dump(cache, open(args.output,'wb'))
            print(f"  [{idx+1}/{len(all_imgs)}] {rate:.2f} img/s | ETA: {eta/60:.1f}min")

    pickle.dump(cache, open(args.output,'wb'))
    print(f"Done! {len(cache)} images cached in {(time.time()-t0)/60:.1f}min")
    rknn.release()

if __name__ == '__main__':
    main()
