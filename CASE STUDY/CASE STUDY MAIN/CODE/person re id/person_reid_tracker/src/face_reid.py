"""Face detection and embedding for owner re-identification."""

from __future__ import annotations

import logging
from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import List, Optional, Tuple

import cv2
import numpy as np

from src.utils import crop_with_padding, max_similarity_to_gallery, normalize_embedding

logger = logging.getLogger(__name__)


@dataclass
class FaceMatchResult:
    similarity: float
    face_detected: bool
    bbox: Optional[Tuple[int, int, int, int]] = None  # xyxy in person crop or frame coords


class FaceEmbedderBackend(ABC):
    @abstractmethod
    def extract(self, image_bgr: np.ndarray) -> List[np.ndarray]:
        """Return list of normalized face embeddings from image."""

    @abstractmethod
    def extract_largest(self, image_bgr: np.ndarray) -> Tuple[Optional[np.ndarray], Optional[Tuple[int, int, int, int]]]:
        """Return embedding and bbox of the largest face."""

    @property
    @abstractmethod
    def name(self) -> str:
        pass


class InsightFaceBackend(FaceEmbedderBackend):
    """InsightFace buffalo_l – preferred when installed."""

    def __init__(
        self,
        device: str = "cpu",
        model_name: str = "buffalo_l",
        det_size: int = 640,
    ):
        try:
            from insightface.app import FaceAnalysis
        except ImportError as exc:
            raise ImportError("insightface not installed") from exc

        providers = self._select_providers(device)
        ctx_id = 0 if "cuda" in device else -1
        if "cuda" in device and "CUDAExecutionProvider" not in providers:
            logger.warning(
                "CUDA device selected, but onnxruntime CUDAExecutionProvider is unavailable; "
                "InsightFace is falling back to CPU."
            )
            ctx_id = -1

        self.app = FaceAnalysis(name=model_name, providers=providers)
        self.app.prepare(ctx_id=ctx_id, det_size=(det_size, det_size))
        logger.info(
            "InsightFace backend loaded: model=%s providers=%s ctx_id=%s det_size=%sx%s",
            model_name,
            providers,
            ctx_id,
            det_size,
            det_size,
        )

    @staticmethod
    def _select_providers(device: str) -> list[str]:
        if "cuda" not in device:
            return ["CPUExecutionProvider"]

        requested = ["CUDAExecutionProvider", "CPUExecutionProvider"]
        try:
            import onnxruntime as ort
        except ImportError:
            logger.warning(
                "onnxruntime is not installed; requesting InsightFace CUDA provider may fail."
            )
            return requested

        available = set(ort.get_available_providers())
        if "CUDAExecutionProvider" not in available:
            logger.warning(
                "CUDAExecutionProvider not available in onnxruntime providers: %s",
                sorted(available),
            )
            return ["CPUExecutionProvider"]
        return requested

    @property
    def name(self) -> str:
        return "insightface"

    def extract(self, image_bgr: np.ndarray) -> List[np.ndarray]:
        faces = self.app.get(image_bgr)
        return [normalize_embedding(f.normed_embedding) for f in faces if f.normed_embedding is not None]

    def extract_largest(self, image_bgr: np.ndarray) -> Tuple[Optional[np.ndarray], Optional[Tuple[int, int, int, int]]]:
        faces = self.app.get(image_bgr)
        if not faces:
            return None, None
        face = max(faces, key=lambda f: (f.bbox[2] - f.bbox[0]) * (f.bbox[3] - f.bbox[1]))
        bbox = tuple(int(v) for v in face.bbox)
        emb = normalize_embedding(face.normed_embedding) if face.normed_embedding is not None else None
        return emb, bbox


class FaceRecognitionBackend(FaceEmbedderBackend):
    """Fallback using face_recognition (dlib ResNet)."""

    def __init__(self):
        try:
            import face_recognition  # noqa: F401
        except ImportError as exc:
            raise ImportError("face_recognition not installed") from exc
        logger.info("face_recognition backend loaded.")

    @property
    def name(self) -> str:
        return "face_recognition"

    def extract(self, image_bgr: np.ndarray) -> List[np.ndarray]:
        import face_recognition

        rgb = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)
        locations = face_recognition.face_locations(rgb, model="hog")
        encodings = face_recognition.face_encodings(rgb, locations)
        return [normalize_embedding(e) for e in encodings]

    def extract_largest(self, image_bgr: np.ndarray) -> Tuple[Optional[np.ndarray], Optional[Tuple[int, int, int, int]]]:
        import face_recognition

        rgb = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)
        locations = face_recognition.face_locations(rgb, model="hog")
        if not locations:
            return None, None
        # top, right, bottom, left -> xyxy
        areas = [(b - t) * (r - l) for t, r, b, l in locations]
        idx = int(np.argmax(areas))
        t, r, b, l = locations[idx]
        encodings = face_recognition.face_encodings(rgb, [locations[idx]])
        if not encodings:
            return None, None
        return normalize_embedding(encodings[0]), (l, t, r, b)


class OpenCVFallbackBackend(FaceEmbedderBackend):
    """
    Minimal fallback: Haar face detector + grayscale histogram embedding.
    Not accurate – install insightface or face_recognition for real use.
    """

    def __init__(self):
        cascade_path = cv2.data.haarcascades + "haarcascade_frontalface_default.xml"
        self.detector = cv2.CascadeClassifier(cascade_path)
        logger.warning(
            "Using OpenCV Haar fallback for face ReID. "
            "Install insightface or face_recognition for reliable results."
        )

    @property
    def name(self) -> str:
        return "opencv_fallback"

    def _embed_face(self, face_bgr: np.ndarray) -> np.ndarray:
        gray = cv2.cvtColor(face_bgr, cv2.COLOR_BGR2GRAY)
        resized = cv2.resize(gray, (64, 64))
        hist = cv2.calcHist([resized], [0], None, [64], [0, 256]).flatten()
        return normalize_embedding(hist.astype(np.float32))

    def extract(self, image_bgr: np.ndarray) -> List[np.ndarray]:
        gray = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY)
        faces = self.detector.detectMultiScale(gray, 1.1, 4, minSize=(40, 40))
        embeddings = []
        for x, y, w, h in faces:
            crop = image_bgr[y : y + h, x : x + w]
            embeddings.append(self._embed_face(crop))
        return embeddings

    def extract_largest(self, image_bgr: np.ndarray) -> Tuple[Optional[np.ndarray], Optional[Tuple[int, int, int, int]]]:
        gray = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY)
        faces = self.detector.detectMultiScale(gray, 1.1, 4, minSize=(40, 40))
        if len(faces) == 0:
            return None, None
        x, y, w, h = max(faces, key=lambda f: f[2] * f[3])
        crop = image_bgr[y : y + h, x : x + w]
        return self._embed_face(crop), (x, y, x + w, y + h)


def create_face_backend(
    device: str = "cpu",
    model_name: str = "buffalo_l",
    det_size: int = 640,
) -> Optional[FaceEmbedderBackend]:
    """Try backends in order: InsightFace, face_recognition, OpenCV fallback."""
    for backend_name, factory in (
        ("InsightFace", lambda: InsightFaceBackend(device, model_name, det_size)),
        ("face_recognition", lambda: FaceRecognitionBackend()),
        ("OpenCV fallback", lambda: OpenCVFallbackBackend()),
    ):
        try:
            return factory()
        except ImportError as exc:
            logger.warning("%s face backend unavailable: %s", backend_name, exc)
        except Exception as exc:
            logger.warning("%s face backend failed to initialize: %s", backend_name, exc)
            continue
    return None


class FaceReID:
    """Face re-identification against enrolled owner gallery."""

    def __init__(
        self,
        device: str = "cpu",
        enabled: bool = True,
        model_name: str = "buffalo_l",
        det_size: int = 640,
    ):
        self.enabled = enabled
        self.backend: Optional[FaceEmbedderBackend] = None
        self.owner_gallery: Optional[np.ndarray] = None

        if enabled:
            self.backend = create_face_backend(device, model_name, det_size)
            if self.backend is None:
                logger.error(
                    "No face ReID backend available. "
                    "Install insightface or face_recognition, or disable use_face_reid."
                )
                self.enabled = False

    def load_owner_gallery(self, face_embeddings: np.ndarray) -> None:
        self.owner_gallery = np.asarray(face_embeddings, dtype=np.float32)

    @property
    def is_ready(self) -> bool:
        return self.enabled and self.backend is not None and self.owner_gallery is not None

    def enroll_from_frame(self, frame: np.ndarray) -> List[np.ndarray]:
        if not self.backend:
            return []
        return self.backend.extract(frame)

    def match_person_crop(
        self,
        frame: np.ndarray,
        person_bbox: np.ndarray,
        upper_body_ratio: float = 0.55,
    ) -> FaceMatchResult:
        """
        Detect face in upper portion of person bbox and compare to owner gallery.
        """
        if not self.is_ready:
            return FaceMatchResult(similarity=0.0, face_detected=False)

        x1, y1, x2, y2 = person_bbox
        h = y2 - y1
        upper_bbox = (x1, y1, x2, y1 + h * upper_body_ratio)
        crop = crop_with_padding(frame, upper_bbox, pad_ratio=0.02)
        if crop is None or crop.size == 0:
            return FaceMatchResult(similarity=0.0, face_detected=False)

        emb, face_bbox = self.backend.extract_largest(crop)
        if emb is None:
            return FaceMatchResult(similarity=0.0, face_detected=False)

        sim = max_similarity_to_gallery(emb, self.owner_gallery)
        # Map face bbox back to frame coordinates
        frame_bbox = None
        if face_bbox is not None:
            cx1, cy1, cx2, cy2 = upper_bbox
            fx1 = int(cx1 + face_bbox[0])
            fy1 = int(cy1 + face_bbox[1])
            fx2 = int(cx1 + face_bbox[2])
            fy2 = int(cy1 + face_bbox[3])
            frame_bbox = (fx1, fy1, fx2, fy2)

        return FaceMatchResult(similarity=sim, face_detected=True, bbox=frame_bbox)
