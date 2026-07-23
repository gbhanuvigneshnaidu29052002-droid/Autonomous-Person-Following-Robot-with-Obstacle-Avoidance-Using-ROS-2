"""Person/body re-identification from full-body crops."""

from __future__ import annotations

import logging
from abc import ABC, abstractmethod
from typing import Optional

import cv2
import numpy as np

from src.utils import crop_with_padding, max_similarity_to_gallery, normalize_embedding

logger = logging.getLogger(__name__)


class BodyEmbedderBackend(ABC):
    @abstractmethod
    def embed(self, body_bgr: np.ndarray) -> np.ndarray:
        pass

    @property
    @abstractmethod
    def name(self) -> str:
        pass


class TorchReIDBackend(BodyEmbedderBackend):
    """OSNet via torchreid – preferred when installed."""

    def __init__(self, device: str = "cpu", model_name: str = "osnet_x0_5"):
        try:
            import torch
            import torchreid
            from torchvision import transforms
        except ImportError as exc:
            raise ImportError("torchreid not installed") from exc

        self.torch = torch
        self.device = torch.device(
            device if device != "cpu" and torch.cuda.is_available() else "cpu"
        )
        self.model = torchreid.models.build_model(
            name=model_name,
            num_classes=1000,
            pretrained=True,
        )
        self.model.eval()
        self.model.to(self.device)
        self.model_name = model_name
        self.transform = transforms.Compose([
            transforms.ToPILImage(),
            transforms.Resize((256, 128)),
            transforms.ToTensor(),
            transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]),
        ])
        logger.info("Torchreid OSNet backend loaded: %s on %s", model_name, self.device)

    @property
    def name(self) -> str:
        return "torchreid_osnet"

    def embed(self, body_bgr: np.ndarray) -> np.ndarray:
        rgb = cv2.cvtColor(body_bgr, cv2.COLOR_BGR2RGB)
        tensor = self.transform(rgb).unsqueeze(0).to(self.device)
        with self.torch.no_grad():
            features = self.model(tensor)
            if isinstance(features, (tuple, list)):
                features = features[0]
        emb = features.detach().cpu().numpy().flatten()
        return normalize_embedding(emb)


class MobileNetFallbackBackend(BodyEmbedderBackend):
    """
    Lightweight torchvision MobileNetV3 feature extractor.
    Works without torchreid; less accurate than OSNet but practical on laptop/RPi.
    """

    def __init__(self, device: str = "cpu"):
        import torch
        from torchvision import models, transforms

        self.torch = torch
        self.device = torch.device(device if device != "cpu" and torch.cuda.is_available() else "cpu")
        weights = models.MobileNet_V3_Small_Weights.DEFAULT
        base = models.mobilenet_v3_small(weights=weights)
        self.model = base.features
        self.model.eval()
        self.model.to(self.device)
        self.transform = transforms.Compose([
            transforms.ToPILImage(),
            transforms.Resize((256, 128)),
            transforms.ToTensor(),
            transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]),
        ])
        logger.info("MobileNetV3 fallback body ReID backend loaded on %s.", self.device)

    @property
    def name(self) -> str:
        return "mobilenet_fallback"

    def embed(self, body_bgr: np.ndarray) -> np.ndarray:
        rgb = cv2.cvtColor(body_bgr, cv2.COLOR_BGR2RGB)
        tensor = self.transform(rgb).unsqueeze(0).to(self.device)
        with self.torch.no_grad():
            features = self.model(tensor)
            pooled = features.mean(dim=[2, 3])
        emb = pooled.cpu().numpy().flatten()
        return normalize_embedding(emb)


def create_body_backend(
    device: str = "cpu",
    model_name: str = "osnet_x0_5",
) -> Optional[BodyEmbedderBackend]:
    for backend_name, factory in (
        ("Torchreid OSNet", lambda: TorchReIDBackend(device, model_name)),
        ("MobileNetV3 fallback", lambda: MobileNetFallbackBackend(device)),
    ):
        try:
            return factory()
        except Exception as exc:
            if backend_name == "Torchreid OSNet":
                logger.warning(
                    "Torchreid OSNet loading failed for %s on %s: %r",
                    model_name,
                    device,
                    exc,
                )
            else:
                logger.warning("%s body ReID backend unavailable: %r", backend_name, exc)
            continue
    return None


class PersonReID:
    """Body/person re-identification against enrolled owner gallery."""

    def __init__(
        self,
        device: str = "cpu",
        enabled: bool = True,
        model_name: str = "osnet_x0_5",
    ):
        self.enabled = enabled
        self.backend: Optional[BodyEmbedderBackend] = None
        self.owner_gallery: Optional[np.ndarray] = None

        if enabled:
            self.backend = create_body_backend(device, model_name)
            if self.backend is None:
                logger.error(
                    "No body ReID backend available. "
                    "Install torch/torchvision or torchreid, or disable use_body_reid."
                )
                self.enabled = False

    def load_owner_gallery(self, body_embeddings: np.ndarray) -> None:
        self.owner_gallery = np.asarray(body_embeddings, dtype=np.float32)

    @property
    def is_ready(self) -> bool:
        return self.enabled and self.backend is not None and self.owner_gallery is not None

    def enroll_from_frame(self, frame: np.ndarray, person_bbox: np.ndarray) -> Optional[np.ndarray]:
        if not self.backend:
            return None
        crop = crop_with_padding(frame, tuple(person_bbox))
        if crop is None or crop.size == 0:
            return None
        if crop.shape[0] < 32 or crop.shape[1] < 16:
            return None
        return self.backend.embed(crop)

    def match_person(
        self,
        frame: np.ndarray,
        person_bbox: np.ndarray,
    ) -> float:
        """Return body similarity score (0–1) against owner gallery."""
        if not self.is_ready:
            return 0.0
        emb = self.enroll_from_frame(frame, person_bbox)
        if emb is None:
            return 0.0
        return max_similarity_to_gallery(emb, self.owner_gallery)
