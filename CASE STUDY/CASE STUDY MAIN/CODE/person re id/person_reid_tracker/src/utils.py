"""Shared utilities: config, paths, device selection, embedding I/O."""

from __future__ import annotations

import logging
from pathlib import Path
from typing import Any, Dict, Optional, Tuple

import cv2
import numpy as np
import yaml

logger = logging.getLogger(__name__)

PROJECT_ROOT = Path(__file__).resolve().parent.parent


def get_project_root() -> Path:
    return PROJECT_ROOT


def load_config(config_path: Optional[str | Path] = None) -> Dict[str, Any]:
    """Load YAML config and resolve path fields relative to project root."""
    if config_path is None:
        config_path = PROJECT_ROOT / "config.yaml"
    else:
        config_path = Path(config_path)
        if not config_path.is_absolute():
            config_path = PROJECT_ROOT / config_path

    if not config_path.exists():
        raise FileNotFoundError(f"Config not found: {config_path}")

    with open(config_path, "r", encoding="utf-8") as f:
        config = yaml.safe_load(f)

    for key in ("embeddings_path", "owner_images_dir", "models_dir", "debug_save_dir"):
        if key in config and config[key]:
            config[key] = str((PROJECT_ROOT / config[key]).resolve())

    return config


def resolve_path(relative: str | Path) -> Path:
    path = Path(relative)
    if path.is_absolute():
        return path
    return (PROJECT_ROOT / path).resolve()


def get_device(use_gpu: bool = True) -> str:
    """Return 'cuda' when requested and available, else 'cpu'."""
    try:
        import torch
    except ImportError:
        logger.warning("PyTorch is not installed; using CPU.")
        return "cpu"

    cuda_available = bool(torch.cuda.is_available())
    logger.info("torch version: %s", getattr(torch, "__version__", "unknown"))
    logger.info("torch.cuda.is_available(): %s", cuda_available)
    logger.info("torch.version.cuda: %s", getattr(torch.version, "cuda", None))

    if cuda_available:
        try:
            logger.info("GPU: %s", torch.cuda.get_device_name(0))
        except Exception as exc:
            logger.warning("CUDA is available but GPU name lookup failed: %s", exc)

    if use_gpu and cuda_available:
        logger.info("Using device: cuda")
        logger.info("CUDA available: True")
        return "cuda"

    if use_gpu and not cuda_available:
        logger.warning("Config requested GPU, but CUDA is unavailable; using CPU.")

    logger.info("Using device: cpu")
    logger.info("CUDA available: %s", cuda_available)
    return "cpu"


def open_camera(
    camera_index: int,
    width: int,
    height: int,
) -> cv2.VideoCapture:
    cap = cv2.VideoCapture(camera_index, cv2.CAP_DSHOW if _is_windows() else cv2.CAP_ANY)
    if not cap.isOpened():
        cap = cv2.VideoCapture(camera_index)
    if not cap.isOpened():
        raise RuntimeError(
            f"Could not open webcam at index {camera_index}. "
            "Check camera_index in config.yaml."
        )
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    return cap


def _is_windows() -> bool:
    import sys
    return sys.platform.startswith("win")


def crop_with_padding(
    frame: np.ndarray,
    bbox: Tuple[float, float, float, float],
    pad_ratio: float = 0.05,
) -> Optional[np.ndarray]:
    """Crop frame region defined by xyxy bbox with optional padding."""
    h, w = frame.shape[:2]
    x1, y1, x2, y2 = bbox
    bw, bh = x2 - x1, y2 - y1
    x1 = max(0, int(x1 - bw * pad_ratio))
    y1 = max(0, int(y1 - bh * pad_ratio))
    x2 = min(w, int(x2 + bw * pad_ratio))
    y2 = min(h, int(y2 + bh * pad_ratio))
    if x2 <= x1 or y2 <= y1:
        return None
    return frame[y1:y2, x1:x2].copy()


def cosine_similarity(a: np.ndarray, b: np.ndarray) -> float:
    """Cosine similarity between two 1-D vectors."""
    a = np.asarray(a, dtype=np.float32).flatten()
    b = np.asarray(b, dtype=np.float32).flatten()
    norm_a = np.linalg.norm(a)
    norm_b = np.linalg.norm(b)
    if norm_a < 1e-8 or norm_b < 1e-8:
        return 0.0
    return float(np.dot(a, b) / (norm_a * norm_b))


def max_similarity_to_gallery(embedding: np.ndarray, gallery: np.ndarray) -> float:
    """Return best cosine similarity against a gallery of embeddings."""
    if gallery is None or len(gallery) == 0:
        return 0.0
    gallery = np.asarray(gallery, dtype=np.float32)
    if gallery.ndim == 1:
        return cosine_similarity(embedding, gallery)
    return max(cosine_similarity(embedding, g) for g in gallery)


def normalize_embedding(embedding: np.ndarray) -> np.ndarray:
    emb = np.asarray(embedding, dtype=np.float32).flatten()
    norm = np.linalg.norm(emb)
    if norm < 1e-8:
        return emb
    return emb / norm


def save_owner_embeddings(
    path: str | Path,
    face_embeddings: list[np.ndarray],
    body_embeddings: list[np.ndarray],
) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = {}
    if face_embeddings:
        payload["face"] = np.stack([normalize_embedding(e) for e in face_embeddings], axis=0)
    if body_embeddings:
        payload["body"] = np.stack([normalize_embedding(e) for e in body_embeddings], axis=0)
    np.savez_compressed(path, **payload)
    logger.info(
        "Saved %d face and %d body embeddings to %s",
        len(face_embeddings),
        len(body_embeddings),
        path,
    )


def load_owner_embeddings(path: str | Path) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
    path = Path(path)
    if not path.exists():
        return None, None
    data = np.load(path)
    face = data["face"] if "face" in data else None
    body = data["body"] if "body" in data else None
    return face, body


def embeddings_exist(path: str | Path) -> bool:
    path = Path(path)
    if not path.exists():
        return False
    try:
        face, body = load_owner_embeddings(path)
        return (face is not None and len(face) > 0) or (body is not None and len(body) > 0)
    except Exception:
        return False


def setup_logging(level: int = logging.INFO) -> None:
    logging.basicConfig(
        level=level,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
        datefmt="%H:%M:%S",
    )


class FPSCounter:
    """Simple rolling FPS estimate."""

    def __init__(self, window: int = 30):
        self.window = window
        self._times: list[float] = []

    def tick(self, dt: float) -> None:
        self._times.append(dt)
        if len(self._times) > self.window:
            self._times.pop(0)

    @property
    def fps(self) -> float:
        if not self._times:
            return 0.0
        mean_dt = sum(self._times) / len(self._times)
        return 1.0 / mean_dt if mean_dt > 0 else 0.0
