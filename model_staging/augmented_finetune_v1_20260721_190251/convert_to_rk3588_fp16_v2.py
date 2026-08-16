#!/usr/bin/env python3
"""Convert the three-head YOLOv5 ONNX model to an RK3588 FP16 RKNN model."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
from rknn.api import RKNN


EXPECTED_OUTPUT_SHAPES = [
    (1, 90, 52, 52),
    (1, 90, 26, 26),
    (1, 90, 13, 13),
]


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def require_success(stage: str, return_code: int) -> None:
    if return_code != 0:
        raise RuntimeError(f"{stage} failed with RKNN return code {return_code}")


def convert(onnx_path: Path, output_path: Path) -> None:
    if not onnx_path.is_file():
        raise FileNotFoundError(f"ONNX model not found: {onnx_path}")
    if output_path.exists():
        raise FileExistsError(f"Refusing to overwrite existing output: {output_path}")

    rknn = RKNN(verbose=True)
    try:
        require_success(
            "config",
            rknn.config(
                mean_values=[[0, 0, 0]],
                std_values=[[255, 255, 255]],
                target_platform="rk3588",
                optimization_level=3,
            ),
        )
        require_success(
            "load_onnx",
            rknn.load_onnx(
                model=str(onnx_path),
                inputs=["images"],
                input_size_list=[[1, 3, 416, 416]],
            ),
        )
        require_success("build", rknn.build(do_quantization=False))
        require_success("export_rknn", rknn.export_rknn(str(output_path)))
    finally:
        rknn.release()


def verify_on_rk3588(model_path: Path) -> list[dict]:
    rknn = RKNN(verbose=False)
    try:
        require_success("load_rknn", rknn.load_rknn(str(model_path)))
        require_success("init_runtime", rknn.init_runtime(target="RK3588"))
        image = np.zeros((416, 416, 3), dtype=np.uint8)
        outputs = rknn.inference(inputs=[image], data_format=["nhwc"])
    finally:
        rknn.release()

    actual_shapes = [tuple(int(value) for value in output.shape) for output in outputs]
    if actual_shapes != EXPECTED_OUTPUT_SHAPES:
        raise RuntimeError(
            f"Unexpected output shapes: {actual_shapes}; expected {EXPECTED_OUTPUT_SHAPES}"
        )

    verification = []
    for output in outputs:
        verification.append(
            {
                "shape": list(output.shape),
                "dtype": str(output.dtype),
                "all_finite": bool(np.isfinite(output).all()),
                "minimum": float(output.min()),
                "maximum": float(output.max()),
            }
        )
    if not all(item["all_finite"] for item in verification):
        raise RuntimeError("RKNN inference produced a non-finite output")
    return verification


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--onnx", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--report", type=Path)
    parser.add_argument("--skip-runtime-check", action="store_true")
    args = parser.parse_args()

    onnx_path = args.onnx.resolve()
    output_path = args.output.resolve()
    report_path = args.report.resolve() if args.report else output_path.with_suffix(".json")
    if report_path.exists():
        raise FileExistsError(f"Refusing to overwrite existing report: {report_path}")

    convert(onnx_path, output_path)
    verification = None
    if not args.skip_runtime_check:
        verification = verify_on_rk3588(output_path)

    report = {
        "source_onnx": str(onnx_path),
        "source_onnx_sha256": sha256(onnx_path),
        "rknn": str(output_path),
        "rknn_sha256": sha256(output_path),
        "rknn_size_bytes": output_path.stat().st_size,
        "target_platform": "rk3588",
        "precision": "FP16 (do_quantization=False)",
        "input_contract": "uint8 NHWC [1,416,416,3], mean=0, std=255",
        "runtime_verification": verification,
    }
    report_path.write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
