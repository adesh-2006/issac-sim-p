"""
vision/hand_tracking.py
=======================
Person 1 — Computer Vision Engineer
Vision-Based Human Teleoperation of a Simulated Manipulator

Responsibilities:
  - Capture live frames from the laptop webcam via OpenCV.
  - Detect hand landmarks with MediaPipe Hand Landmarker.
  - Compute a stable 2D palm-center reference point.
  - Normalize image coordinates to [0.0, 1.0] (top-left origin).
  - Detect thumb-index pinch using a hand-size-relative distance.
  - Apply hysteresis so PINCH/OPEN state does not flicker.
  - Apply exponential moving average (EMA) smoothing on hand center.
  - Send valid vision packets (schema v1) to UDP 127.0.0.1:5005 at ~20–30 Hz.
  - Handle: webcam unavailable, no hand detected, program shutdown.

Protocol reference: PROJECT_SPEC.md §7.1
Usage:
    python hand_tracking.py [--camera 0] [--host 127.0.0.1] [--port 5005]
                            [--target-fps 30] [--show] [--no-show]
"""

from __future__ import annotations

import argparse
import json
import logging
import math
import socket
import sys
import time
from dataclasses import dataclass, field
from enum import Enum
from typing import Optional, Tuple

import cv2

# ---------------------------------------------------------------------------
# Optional MediaPipe import — fail with a clear message if not installed
# ---------------------------------------------------------------------------
try:
    import mediapipe as mp
    from mediapipe.framework.formats import landmark_pb2  # noqa: F401 (type checking)
    _MP_AVAILABLE = True
except ImportError:
    _MP_AVAILABLE = False

# ---------------------------------------------------------------------------
# Logging
# ---------------------------------------------------------------------------
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger("vision")

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

# MediaPipe landmark indices (hand_landmarks)
# https://developers.google.com/mediapipe/solutions/vision/hand_landmarker
LM_WRIST       = 0
LM_THUMB_CMC   = 1
LM_THUMB_MCP   = 2
LM_THUMB_IP    = 3
LM_THUMB_TIP   = 4
LM_INDEX_MCP   = 5
LM_INDEX_PIP   = 6
LM_INDEX_DIP   = 7
LM_INDEX_TIP   = 8
LM_MIDDLE_MCP  = 9
LM_RING_MCP    = 13
LM_PINKY_MCP   = 17

# Palm-center computed as the mean of these five base landmarks
PALM_LANDMARKS = [LM_WRIST, LM_INDEX_MCP, LM_MIDDLE_MCP, LM_RING_MCP, LM_PINKY_MCP]

# Pinch measured as Euclidean distance between thumb-tip and index-tip,
# normalised by the diagonal of the palm bounding box (hand-size-relative).
PINCH_TIP_A = LM_THUMB_TIP
PINCH_TIP_B = LM_INDEX_TIP

# Protocol version
PROTOCOL_VERSION = 1

# ---------------------------------------------------------------------------
# Enums
# ---------------------------------------------------------------------------

class GestureState(str, Enum):
    PINCH   = "PINCH"
    OPEN    = "OPEN"
    UNKNOWN = "UNKNOWN"

# ---------------------------------------------------------------------------
# Data classes
# ---------------------------------------------------------------------------

@dataclass
class TrackerConfig:
    # Camera
    camera_index: int = 0
    target_fps: int   = 30          # webcam capture target

    # UDP
    host: str = "127.0.0.1"
    port: int = 5005

    # MediaPipe
    min_detection_confidence: float = 0.6
    min_tracking_confidence:  float = 0.5
    max_num_hands:            int   = 1

    # EMA smoothing (alpha=1.0 means no smoothing)
    smoothing_alpha: float = 0.4    # lower → more smoothing, higher → more responsive

    # Pinch thresholds (fraction of palm diagonal)
    pinch_close_threshold: float = 0.12   # below this → PINCH
    pinch_open_threshold:  float = 0.18   # above this → OPEN (hysteresis gap)

    # Display
    show_preview: bool = True

    # Confidence proxy: MediaPipe presence score available for landmark_pb2
    # We use the mean presence score across all landmarks as our confidence.


@dataclass
class HandState:
    """Mutable runtime state for the tracker."""
    seq:       int         = 0
    gesture:   GestureState = GestureState.UNKNOWN
    smooth_x:  Optional[float] = None
    smooth_y:  Optional[float] = None


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------

def _lm_xy(lm_list, idx: int) -> Tuple[float, float]:
    """Return (x, y) for landmark index; both in [0,1] normalised image space."""
    lm = lm_list[idx]
    return lm.x, lm.y


def compute_palm_center(lm_list) -> Tuple[float, float]:
    """
    Average of five palm-base landmarks:
        wrist (0), index-MCP (5), middle-MCP (9), ring-MCP (13), pinky-MCP (17)

    Returns normalised (x, y) in [0, 1] image space (top-left origin).
    """
    xs = [lm_list[i].x for i in PALM_LANDMARKS]
    ys = [lm_list[i].y for i in PALM_LANDMARKS]
    return sum(xs) / len(xs), sum(ys) / len(ys)


def compute_palm_diagonal(lm_list) -> float:
    """
    Bounding-box diagonal of the palm-base landmarks, in normalised image units.
    Used to scale the pinch distance so it is hand-size-relative.
    """
    xs = [lm_list[i].x for i in PALM_LANDMARKS]
    ys = [lm_list[i].y for i in PALM_LANDMARKS]
    dx = max(xs) - min(xs)
    dy = max(ys) - min(ys)
    diag = math.hypot(dx, dy)
    # Guard against degenerate (too-small) hand detections
    return max(diag, 1e-6)


def compute_pinch_distance_normalized(lm_list) -> float:
    """
    Euclidean distance between thumb-tip and index-tip, divided by the palm
    diagonal.  This makes the threshold hand-size-relative and camera-
    distance-independent.
    """
    tx, ty = _lm_xy(lm_list, PINCH_TIP_A)
    ix, iy = _lm_xy(lm_list, PINCH_TIP_B)
    raw    = math.hypot(tx - ix, ty - iy)
    return raw / compute_palm_diagonal(lm_list)


def compute_confidence(hand_landmarks) -> float:
    """
    Confidence proxy: mean of the presence scores across all 21 landmarks.
    MediaPipe exposes `presence_score` per landmark (0.0–1.0) in the new
    Tasks API.  For the legacy Hands solution, where presence_score may be
    absent, fall back to 1.0 if tracking was reported successful.
    """
    scores = []
    for lm in hand_landmarks.landmark:
        sc = getattr(lm, "presence", None)
        if sc is not None:
            scores.append(float(sc))
    if scores:
        return max(0.0, min(1.0, sum(scores) / len(scores)))
    # Fallback: landmark list is present → tracking succeeded
    return 1.0


# ---------------------------------------------------------------------------
# EMA smoothing
# ---------------------------------------------------------------------------

def ema_update(prev: Optional[float], new_val: float, alpha: float) -> float:
    """Exponential moving average.  alpha=1 → no smoothing."""
    if prev is None:
        return new_val
    return alpha * new_val + (1.0 - alpha) * prev


# ---------------------------------------------------------------------------
# Pinch hysteresis state machine
# ---------------------------------------------------------------------------

def update_gesture(
    current: GestureState,
    pinch_dist: float,
    cfg: TrackerConfig,
) -> GestureState:
    """
    Hysteresis prevents rapid flickering between PINCH and OPEN:
      - Transition to PINCH only when distance drops below pinch_close_threshold.
      - Transition back to OPEN only when distance rises above pinch_open_threshold.
      - Stay in the current PINCH/OPEN state when the distance is in the hysteresis
        band between the two thresholds.
    """
    if pinch_dist < cfg.pinch_close_threshold:
        return GestureState.PINCH
    if pinch_dist > cfg.pinch_open_threshold:
        return GestureState.OPEN
    # Hysteresis band — keep current state (PINCH or OPEN)
    if current == GestureState.UNKNOWN:
        # Default to OPEN if we have no prior state
        return GestureState.OPEN
    return current


# ---------------------------------------------------------------------------
# Packet construction
# ---------------------------------------------------------------------------

def build_packet_tracking(
    seq: int,
    hand_x: float,
    hand_y: float,
    pinch_dist: float,
    gesture: GestureState,
    confidence: float,
) -> dict:
    """Vision packet when tracking is valid (§7.1)."""
    return {
        "version":      PROTOCOL_VERSION,
        "seq":          seq,
        "timestamp_ms": int(time.time() * 1000),
        "tracking":     True,
        "confidence":   round(confidence, 4),
        "hand": {
            "x":              round(hand_x, 6),
            "y":              round(hand_y, 6),
            "pinch_distance": round(pinch_dist, 6),
        },
        "gesture": gesture.value,
    }


def build_packet_no_hand(seq: int) -> dict:
    """Vision packet when no valid hand is detected (§7.1)."""
    return {
        "version":      PROTOCOL_VERSION,
        "seq":          seq,
        "timestamp_ms": int(time.time() * 1000),
        "tracking":     False,
        "confidence":   0.0,
        "hand":         None,
        "gesture":      GestureState.UNKNOWN.value,
    }


def validate_packet(pkt: dict) -> bool:
    """
    Sanity-check a packet before sending.
    Returns False and logs a warning if anything is invalid.
    """
    if pkt["tracking"]:
        h = pkt.get("hand", {}) or {}
        x, y = h.get("x"), h.get("y")
        pd   = h.get("pinch_distance")
        if not (0.0 <= x <= 1.0 and 0.0 <= y <= 1.0):
            log.warning("Packet validation failed: coordinates out of [0,1]: x=%s y=%s", x, y)
            return False
        if not math.isfinite(x) or not math.isfinite(y):
            log.warning("Packet validation failed: non-finite coordinate")
            return False
        if pd is not None and not math.isfinite(pd):
            log.warning("Packet validation failed: non-finite pinch_distance")
            return False
    return True


# ---------------------------------------------------------------------------
# Overlay drawing
# ---------------------------------------------------------------------------

_MP_DRAWING         = None
_MP_DRAWING_STYLES  = None
_MP_HANDS_CONNS     = None

def _ensure_drawing_utils():
    global _MP_DRAWING, _MP_DRAWING_STYLES, _MP_HANDS_CONNS
    if _MP_DRAWING is None and _MP_AVAILABLE:
        _MP_DRAWING        = mp.solutions.drawing_utils
        _MP_DRAWING_STYLES = mp.solutions.drawing_styles
        _MP_HANDS_CONNS    = mp.solutions.hands.HAND_CONNECTIONS


def draw_overlay(
    frame,
    hand_landmarks,
    palm_x: float,
    palm_y: float,
    pinch_dist: float,
    gesture: GestureState,
    confidence: float,
    state_label: str,
    seq: int,
    cfg: TrackerConfig,
):
    """Draw landmarks, palm-center dot, pinch indicator, and HUD text."""
    _ensure_drawing_utils()
    h, w = frame.shape[:2]

    # --- Landmarks & connections ---
    if _MP_DRAWING and hand_landmarks:
        _MP_DRAWING.draw_landmarks(
            frame,
            hand_landmarks,
            _MP_HANDS_CONNS,
            _MP_DRAWING_STYLES.get_default_hand_landmarks_style(),
            _MP_DRAWING_STYLES.get_default_hand_connections_style(),
        )

    # --- Palm center dot ---
    cx, cy = int(palm_x * w), int(palm_y * h)
    cv2.circle(frame, (cx, cy), 10, (0, 255, 100), -1)
    cv2.circle(frame, (cx, cy), 12, (255, 255, 255), 2)

    # --- Pinch tips line ---
    if hand_landmarks:
        lm = hand_landmarks.landmark
        t_px = (int(lm[PINCH_TIP_A].x * w), int(lm[PINCH_TIP_A].y * h))
        i_px = (int(lm[PINCH_TIP_B].x * w), int(lm[PINCH_TIP_B].y * h))
        colour = (0, 100, 255) if gesture == GestureState.PINCH else (180, 180, 180)
        cv2.line(frame, t_px, i_px, colour, 2)
        cv2.circle(frame, t_px, 7, (0, 200, 255), -1)
        cv2.circle(frame, i_px, 7, (0, 255, 200), -1)

    # --- HUD ---
    def _txt(text, row, colour=(255, 255, 255)):
        cv2.putText(frame, text, (10, 28 + row * 26),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(frame, text, (10, 28 + row * 26),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, colour, 1, cv2.LINE_AA)

    gesture_colour = (0, 100, 255) if gesture == GestureState.PINCH else (0, 255, 100)
    _txt(f"State   : {state_label}", 0, (255, 220, 60))
    _txt(f"Gesture : {gesture.value}", 1, gesture_colour)
    _txt(f"Hand XY : ({palm_x:.3f}, {palm_y:.3f})", 2)
    _txt(f"Pinch d : {pinch_dist:.3f}  conf={confidence:.2f}", 3)
    _txt(f"Seq     : {seq}", 4, (160, 160, 160))
    _txt("ESC to quit", 5, (180, 80, 80))

    return frame


def draw_no_hand_overlay(frame, seq: int):
    """Minimal overlay when no hand is detected."""
    cv2.putText(frame, "NO HAND DETECTED", (10, 35),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 0), 4, cv2.LINE_AA)
    cv2.putText(frame, "NO HAND DETECTED", (10, 35),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 60, 255), 2, cv2.LINE_AA)
    cv2.putText(frame, f"seq={seq}", (10, 65),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (140, 140, 140), 1, cv2.LINE_AA)
    return frame


# ---------------------------------------------------------------------------
# Main tracker class
# ---------------------------------------------------------------------------

class HandTracker:
    """
    Orchestrates webcam capture, MediaPipe hand tracking, gesture detection,
    smoothing, overlay rendering, and UDP packet emission.
    """

    def __init__(self, cfg: TrackerConfig):
        self.cfg   = cfg
        self.state = HandState()
        self._sock: Optional[socket.socket] = None
        self._cap:  Optional[cv2.VideoCapture] = None
        self._hands = None   # MediaPipe Hands solution

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def start(self):
        """Open webcam, initialise MediaPipe, open UDP socket."""
        if not _MP_AVAILABLE:
            log.error(
                "MediaPipe is not installed. Run:  pip install mediapipe\n"
                "Then re-run this script."
            )
            sys.exit(1)

        # --- Webcam ---
        log.info("Opening webcam index=%d …", self.cfg.camera_index)
        self._cap = cv2.VideoCapture(self.cfg.camera_index)
        if not self._cap.isOpened():
            log.error(
                "Cannot open webcam at index %d. "
                "Check that the webcam is connected and not in use by another app.",
                self.cfg.camera_index,
            )
            sys.exit(1)

        # Attempt to set capture FPS (best-effort; hardware may override)
        self._cap.set(cv2.CAP_PROP_FPS, self.cfg.target_fps)
        actual_fps = self._cap.get(cv2.CAP_PROP_FPS)
        log.info("Webcam opened. Reported FPS: %.1f", actual_fps)

        # --- MediaPipe ---
        log.info("Initialising MediaPipe Hands …")
        self._hands = mp.solutions.hands.Hands(
            static_image_mode=False,
            max_num_hands=self.cfg.max_num_hands,
            min_detection_confidence=self.cfg.min_detection_confidence,
            min_tracking_confidence=self.cfg.min_tracking_confidence,
        )

        # --- UDP socket ---
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        log.info("UDP socket ready → %s:%d", self.cfg.host, self.cfg.port)

    def stop(self):
        """Release all resources cleanly."""
        log.info("Shutting down vision process …")
        if self._cap is not None:
            self._cap.release()
        if self._hands is not None:
            self._hands.close()
        if self._sock is not None:
            self._sock.close()
        if self.cfg.show_preview:
            cv2.destroyAllWindows()
        log.info("Vision process stopped.")

    # ------------------------------------------------------------------
    # Main loop
    # ------------------------------------------------------------------

    def run(self):
        """Main capture-process-send loop. Runs until ESC or KeyboardInterrupt."""
        self.start()

        frame_interval = 1.0 / max(self.cfg.target_fps, 1)
        last_send_time = 0.0

        try:
            while True:
                ret, frame = self._cap.read()
                if not ret:
                    log.warning("Webcam read failed — frame dropped.")
                    time.sleep(0.02)
                    continue

                # Flip horizontally for mirror-view (natural for the operator)
                frame = cv2.flip(frame, 1)

                now = time.monotonic()
                tracking, pkt, display_frame = self._process_frame(frame)

                # Rate-limit UDP sends to target FPS
                if now - last_send_time >= frame_interval:
                    self._send(pkt)
                    last_send_time = now

                if self.cfg.show_preview:
                    cv2.imshow("HandPilot — Vision", display_frame)
                    key = cv2.waitKey(1) & 0xFF
                    if key == 27:  # ESC
                        log.info("ESC key pressed — exiting.")
                        break

        except KeyboardInterrupt:
            log.info("KeyboardInterrupt received — exiting.")
        finally:
            self.stop()

    # ------------------------------------------------------------------
    # Per-frame processing
    # ------------------------------------------------------------------

    def _process_frame(self, frame) -> Tuple[bool, dict, any]:
        """
        Process a single BGR frame.

        Returns:
            tracking      : bool  — whether a valid hand was found
            packet        : dict  — ready-to-send vision packet
            display_frame : frame — annotated for display
        """
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        rgb.flags.writeable = False
        results = self._hands.process(rgb)
        rgb.flags.writeable = True
        display_frame = frame.copy()

        if not results.multi_hand_landmarks:
            # ---- No hand detected ----------------------------------------
            self.state.gesture  = GestureState.UNKNOWN
            self.state.smooth_x = None
            self.state.smooth_y = None
            self.state.seq     += 1

            pkt = build_packet_no_hand(self.state.seq)
            display_frame = draw_no_hand_overlay(display_frame, self.state.seq)
            return False, pkt, display_frame

        # ---- Hand detected -----------------------------------------------
        hand_lm = results.multi_hand_landmarks[0]
        lm_list = hand_lm.landmark

        # Palm center
        raw_x, raw_y = compute_palm_center(lm_list)

        # EMA smoothing
        self.state.smooth_x = ema_update(self.state.smooth_x, raw_x, self.cfg.smoothing_alpha)
        self.state.smooth_y = ema_update(self.state.smooth_y, raw_y, self.cfg.smoothing_alpha)
        smooth_x = self.state.smooth_x
        smooth_y = self.state.smooth_y

        # Clamp to [0, 1] (rare but possible near frame edges)
        smooth_x = max(0.0, min(1.0, smooth_x))
        smooth_y = max(0.0, min(1.0, smooth_y))

        # Pinch distance (hand-size-relative)
        pinch_dist = compute_pinch_distance_normalized(lm_list)

        # Gesture with hysteresis
        self.state.gesture = update_gesture(
            self.state.gesture, pinch_dist, self.cfg
        )

        # Confidence proxy
        confidence = compute_confidence(hand_lm)

        self.state.seq += 1
        pkt = build_packet_tracking(
            seq=self.state.seq,
            hand_x=smooth_x,
            hand_y=smooth_y,
            pinch_dist=pinch_dist,
            gesture=self.state.gesture,
            confidence=confidence,
        )

        if not validate_packet(pkt):
            # Fall back to no-hand packet if validation fails
            pkt = build_packet_no_hand(self.state.seq)
            display_frame = draw_no_hand_overlay(display_frame, self.state.seq)
            return False, pkt, display_frame

        display_frame = draw_overlay(
            frame=display_frame,
            hand_landmarks=hand_lm,
            palm_x=smooth_x,
            palm_y=smooth_y,
            pinch_dist=pinch_dist,
            gesture=self.state.gesture,
            confidence=confidence,
            state_label="TRACKING",
            seq=self.state.seq,
            cfg=self.cfg,
        )
        return True, pkt, display_frame

    # ------------------------------------------------------------------
    # UDP send
    # ------------------------------------------------------------------

    def _send(self, pkt: dict):
        """Serialise packet as UTF-8 JSON and send via UDP."""
        try:
            data = json.dumps(pkt, separators=(",", ":")).encode("utf-8")
            self._sock.sendto(data, (self.cfg.host, self.cfg.port))
            log.debug("→ seq=%d tracking=%s", pkt["seq"], pkt["tracking"])
        except OSError as exc:
            log.error("UDP send error: %s", exc)


# ---------------------------------------------------------------------------
# CLI entry point
# ---------------------------------------------------------------------------

def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="HandPilot Vision — hand tracking and gesture sender"
    )
    p.add_argument("--camera",     type=int,   default=0,           help="Webcam device index (default: 0)")
    p.add_argument("--host",       type=str,   default="127.0.0.1", help="UDP destination host")
    p.add_argument("--port",       type=int,   default=5005,        help="UDP destination port")
    p.add_argument("--target-fps", type=int,   default=30,          help="Target capture/send rate")
    p.add_argument("--alpha",      type=float, default=0.4,         help="EMA smoothing alpha (0<α≤1)")
    p.add_argument("--show",       action="store_true",  default=True,  help="Show preview window (default)")
    p.add_argument("--no-show",    action="store_false", dest="show",   help="Disable preview window")
    p.add_argument("--debug",      action="store_true",  default=False, help="Enable DEBUG logging")
    return p.parse_args()


def main():
    args = parse_args()

    if args.debug:
        logging.getLogger().setLevel(logging.DEBUG)

    cfg = TrackerConfig(
        camera_index=args.camera,
        host=args.host,
        port=args.port,
        target_fps=args.target_fps,
        smoothing_alpha=args.alpha,
        show_preview=args.show,
    )

    log.info("HandPilot Vision starting …")
    log.info("  Camera   : %d", cfg.camera_index)
    log.info("  UDP dest : %s:%d", cfg.host, cfg.port)
    log.info("  FPS tgt  : %d", cfg.target_fps)
    log.info("  EMA α    : %.2f", cfg.smoothing_alpha)

    tracker = HandTracker(cfg)
    tracker.run()


if __name__ == "__main__":
    main()
