#!/usr/bin/env python3
"""Print GPU and computer-vision dependency diagnostics."""

from __future__ import annotations

import importlib
import sys


def import_status(module_name: str) -> tuple[bool, object | None, str | None]:
    try:
        module = importlib.import_module(module_name)
    except Exception as exc:
        return False, None, repr(exc)
    return True, module, None


def version_of(module: object) -> str:
    return str(getattr(module, "__version__", "unknown"))


def main() -> None:
    print(f"Python executable: {sys.executable}")
    print(f"Python version: {sys.version}")

    ok, torch, error = import_status("torch")
    if ok:
        print(f"torch version: {version_of(torch)}")
        cuda_available = bool(torch.cuda.is_available())
        print(f"torch.cuda.is_available(): {cuda_available}")
        print(f"torch.version.cuda: {getattr(torch.version, 'cuda', None)}")
        if cuda_available:
            print(f"GPU name: {torch.cuda.get_device_name(0)}")
        else:
            print("GPU name: NO GPU")
    else:
        print(f"torch import: FAILED {error}")
        print("torch.cuda.is_available(): unavailable")
        print("torch.version.cuda: unavailable")
        print("GPU name: unavailable")

    ok, ort, error = import_status("onnxruntime")
    if ok:
        providers = ort.get_available_providers()
        print(f"onnxruntime available providers: {providers}")
        print(f"CUDAExecutionProvider available: {'CUDAExecutionProvider' in providers}")
    else:
        print(f"onnxruntime import: FAILED {error}")
        print("onnxruntime available providers: unavailable")
        print("CUDAExecutionProvider available: False")

    ok, cv2, error = import_status("cv2")
    print(f"cv2 version: {version_of(cv2) if ok else 'FAILED ' + str(error)}")

    ok, ultralytics, error = import_status("ultralytics")
    print(f"ultralytics version: {version_of(ultralytics) if ok else 'FAILED ' + str(error)}")

    ok, _insightface, error = import_status("insightface")
    print(f"insightface import: {'OK' if ok else 'FAILED ' + str(error)}")

    ok, _torchreid, error = import_status("torchreid")
    print(f"torchreid import: {'OK' if ok else 'FAILED ' + str(error)}")


if __name__ == "__main__":
    main()
