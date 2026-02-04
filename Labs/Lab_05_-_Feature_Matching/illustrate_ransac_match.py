#!/usr/bin/env python3
import argparse
import os
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np


IMG_EXTS = {".png", ".jpg", ".jpeg", ".bmp", ".tif", ".tiff", ".webp"}


@dataclass(frozen=True)
class AlgoSpec:
    name: str
    color_bgr: Tuple[int, int, int]


def list_images(directory: str) -> List[str]:
    paths = []
    for fn in os.listdir(directory):
        ext = os.path.splitext(fn)[1].lower()
        if ext in IMG_EXTS:
            paths.append(os.path.join(directory, fn))
    paths.sort()
    return paths


def try_create_sift():
    # Most modern OpenCV: cv2.SIFT_create()
    if hasattr(cv2, "SIFT_create"):
        try:
            return cv2.SIFT_create()
        except Exception:
            return None
    return None


def try_create_surf(hessian_threshold: float = 400.0):
    # SURF is usually in xfeatures2d (opencv-contrib) and may require nonfree build.
    if hasattr(cv2, "xfeatures2d") and hasattr(cv2.xfeatures2d, "SURF_create"):
        try:
            return cv2.xfeatures2d.SURF_create(hessianThreshold=hessian_threshold)
        except Exception:
            return None
    return None


def create_orb(nfeatures: int = 2000):
    return cv2.ORB_create(nfeatures=nfeatures)


def to_gray(img_bgr: np.ndarray) -> np.ndarray:
    return cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)


def resize_keep_aspect(img: np.ndarray, scale: float) -> np.ndarray:
    if scale == 1.0:
        return img
    h, w = img.shape[:2]
    nh = max(1, int(round(h * scale)))
    nw = max(1, int(round(w * scale)))
    return cv2.resize(img, (nw, nh), interpolation=cv2.INTER_AREA)


def pad_to_height(img: np.ndarray, height: int) -> np.ndarray:
    h, w = img.shape[:2]
    if h == height:
        return img
    if h > height:
        # Should not happen if we normalize by resizing, but handle anyway.
        return cv2.resize(img, (w, height), interpolation=cv2.INTER_AREA)
    pad = height - h
    return cv2.copyMakeBorder(img, 0, pad, 0, 0, borderType=cv2.BORDER_CONSTANT, value=(0, 0, 0))


def norm_for_matching(algo_name: str) -> int:
    # ORB uses binary descriptors => Hamming
    # SIFT/SURF use float => L2
    return cv2.NORM_HAMMING if algo_name == "ORB" else cv2.NORM_L2


def compute_kp_desc(detector, img_gray: np.ndarray):
    # detectAndCompute returns (keypoints, descriptors)
    kps, desc = detector.detectAndCompute(img_gray, None)
    if desc is None:
        # Normalize to empty to simplify downstream code
        return [], None
    return kps, desc


def match_desc(algo_name: str, desc1, desc2, ratio_test: float = 0.75):
    if desc1 is None or desc2 is None:
        return []

    # BFMatcher with KNN
    bf = cv2.BFMatcher(norm_for_matching(algo_name), crossCheck=False)
    try:
        knn = bf.knnMatch(desc1, desc2, k=2)
    except cv2.error:
        return []

    good = []
    for m_n in knn:
        if len(m_n) < 2:
            continue
        m, n = m_n
        if m.distance < ratio_test * n.distance:
            good.append(m)
    return good

def ransac_filter_matches_homography(
    kps1: List[cv2.KeyPoint],
    kps2: List[cv2.KeyPoint],
    matches: List[cv2.DMatch],
    thresh_px: float,
    max_iters: int = 2000,
    confidence: float = 0.995,
):
    if len(matches) < 4:
        return [], None, None

    pts1 = np.float32([kps1[m.queryIdx].pt for m in matches]).reshape(-1, 1, 2)
    pts2 = np.float32([kps2[m.trainIdx].pt for m in matches]).reshape(-1, 1, 2)

    H, mask = cv2.findHomography(
        pts1,
        pts2,
        method=cv2.RANSAC,
        ransacReprojThreshold=thresh_px,
        maxIters=max_iters,
        confidence=confidence,
    )
    if mask is None:
        return [], H, None

    mask = mask.ravel().astype(bool)
    inliers = [m for m, ok in zip(matches, mask) if ok]
    return inliers, H, mask


def ransac_filter_matches_fundamental(
    kps1: List[cv2.KeyPoint],
    kps2: List[cv2.KeyPoint],
    matches: List[cv2.DMatch],
    thresh_px: float,
    confidence: float = 0.995,
    max_iters: int = 2000,
):
    # Fundamental matrix needs >= 7 (7-point) but OpenCV will handle >= 8 robustly too.
    if len(matches) < 8:
        return [], None, None

    pts1 = np.float32([kps1[m.queryIdx].pt for m in matches]).reshape(-1, 2)
    pts2 = np.float32([kps2[m.trainIdx].pt for m in matches]).reshape(-1, 2)

    F, mask = cv2.findFundamentalMat(
        pts1,
        pts2,
        method=cv2.FM_RANSAC,
        ransacReprojThreshold=thresh_px,
        confidence=confidence,
        maxIters=max_iters,
    )
    if mask is None:
        return [], F, None

    mask = mask.ravel().astype(bool)
    inliers = [m for m, ok in zip(matches, mask) if ok]
    return inliers, F, mask


def draw_kps_and_matches(
    left_bgr: np.ndarray,
    right_bgr: np.ndarray,
    results: Dict[str, Dict],
    algo_specs: Dict[str, AlgoSpec],
    show_keypoints: bool = True,
    show_matches: bool = True,
) -> np.ndarray:
    """
    Build a side-by-side canvas and draw:
      - keypoints on each side (colored per algorithm)
      - match lines across the seam (colored per algorithm)
    results[algo] contains: kps1, kps2, matches
    """
    lh, lw = left_bgr.shape[:2]
    rh, rw = right_bgr.shape[:2]
    H = max(lh, rh)

    left = pad_to_height(left_bgr, H)
    right = pad_to_height(right_bgr, H)

    canvas = np.hstack([left.copy(), right.copy()])
    x_off = left.shape[1]

    if show_keypoints:
        for algo, pack in results.items():
            spec = algo_specs[algo]
            kps1 = pack.get("kps1", [])
            kps2 = pack.get("kps2", [])
            # Draw as small circles for clarity when overlaying multiple algos.
            for kp in kps1:
                x, y = kp.pt
                cv2.circle(canvas, (int(round(x)), int(round(y))), 2, spec.color_bgr, 1, cv2.LINE_AA)
            for kp in kps2:
                x, y = kp.pt
                cv2.circle(canvas, (x_off + int(round(x)), int(round(y))), 2, spec.color_bgr, 1, cv2.LINE_AA)

    if show_matches:
        for algo, pack in results.items():
            spec = algo_specs[algo]
            kps1 = pack.get("kps1", [])
            kps2 = pack.get("kps2", [])
            matches = pack.get("matches", [])
            for m in matches:
                if m.queryIdx >= len(kps1) or m.trainIdx >= len(kps2):
                    continue
                x1, y1 = kps1[m.queryIdx].pt
                x2, y2 = kps2[m.trainIdx].pt
                p1 = (int(round(x1)), int(round(y1)))
                p2 = (x_off + int(round(x2)), int(round(y2)))
                cv2.line(canvas, p1, p2, spec.color_bgr, 1, cv2.LINE_AA)

    # draw seam line
    cv2.line(canvas, (x_off, 0), (x_off, H - 1), (80, 80, 80), 1, cv2.LINE_AA)
    return canvas


def put_hud(
    img: np.ndarray,
    left_name: str,
    right_name: str,
    left_idx: int,
    right_idx: int,
    total: int,
    enabled: Dict[str, bool],
    active_side: str,
    counts: Dict[str, Tuple[int, int, int]],
    geom_mode: str,
    ransac_thresh_px : float,
    show_help: bool,
):
    # semi-transparent bar
    overlay = img.copy()

    if True:        
        cv2.rectangle(overlay, (0, 0), (img.shape[1], 92), (0, 0, 0), -1)
        img[:] = cv2.addWeighted(overlay, 0.55, img, 0.45, 0)

    def line(x, y, text, scale=0.3):
        cv2.putText(img, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, scale, (240, 240, 240), 1, cv2.LINE_AA)
        
    if True:        
        line(10, 22, f"Left [{left_idx+1}/{total}]  {left_name}")
        line(img.shape[1] - 200, 22, f"Geom: {geom_mode}", 0.55)
        line(10, 42, f"Right[{right_idx+1}/{total}]  {right_name}")
        line(img.shape[1] - 200, 42, f"thr={ransac_thresh_px:.1f}px", 0.55)
        en = "  ".join([f"{k}:{'ON' if v else 'off'}" for k, v in enabled.items()])
        line(10, 62, f"Enabled: {en}   Active: {active_side}")

        # status = f"Geom: {geom_mode}  thr={ransac_thresh_px:.1f}px"
        # cv2.putText(canvas, status, (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 0), 3, cv2.LINE_AA)
        # cv2.putText(canvas, status, (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (240, 240, 240), 1, cv2.LINE_AA)
        
        # counts per algo
        pieces = []
        for algo, (k1, k2, m) in counts.items():
            pieces.append(f"{algo} kp(L/R)={k1}/{k2} matches={m}")
            line(10, 82, " | ".join(pieces) if pieces else "No features computed.")

    if show_help:
        help_lines = [
            "ESC quit | Tab switch active | A/D left prev/next | J/L right prev/next | arrows active prev/next",
            "1 ORB | 2 SIFT | 3 SURF | G geom OFF/H/F | [ ] thresh | M recompute | R reset | H help",
        ]
        overlay2 = img.copy()
        h = 56
        y0 = img.shape[0] - h
        cv2.rectangle(overlay2, (0, y0), (img.shape[1], img.shape[0]), (0, 0, 0), -1)
        img[:] = cv2.addWeighted(overlay2, 0.55, img, 0.45, 0)
        line(10, img.shape[0] - 32, help_lines[0], 0.55)
        line(10, img.shape[0] - 10, help_lines[1], 0.55)

def key_is(key: int, chars: str) -> bool:
    # key from waitKeyEx can include higher bits; for normal chars it matches ord().
    if key < 0:
        return False
    c = key & 0xFF
    return c in [ord(x) for x in chars]

def imshow_resized(win: str, img: np.ndarray) -> None:
    """
    Display img scaled to the current window size (aspect preserved).
    This avoids white space when the user resizes a WINDOW_NORMAL window.
    """
    # getWindowImageRect returns (x, y, w, h)
    x, y, w, h = cv2.getWindowImageRect(win)

    # During initial creation/minimize, w/h can be 0
    if w <= 1 or h <= 1:
        cv2.imshow(win, img)
        return

    ih, iw = img.shape[:2]
    if iw <= 0 or ih <= 0:
        cv2.imshow(win, img)
        return

    # Fit-to-window scale (letterbox if aspect differs)
    scale = min(w / iw, h / ih)
    nw = max(1, int(round(iw * scale)))
    nh = max(1, int(round(ih * scale)))

    if (nw, nh) == (iw, ih):
        cv2.imshow(win, img)
        return

    disp = cv2.resize(img, (nw, nh), interpolation=cv2.INTER_AREA)
    cv2.imshow(win, disp)

def build_active_tags(enabled: Dict[str, bool], detectors: Dict[str, object]) -> List[str]:
    """
    Return tags in a stable order. 'SUP' denotes SURF (per user naming).
    Only include tags for algos that are enabled AND available (detector not None).
    """
    tags = []
    if enabled.get("SIFT", False) and detectors.get("SIFT") is not None:
        tags.append("SIFT")
    if enabled.get("ORB", False) and detectors.get("ORB") is not None:
        tags.append("ORB")
    if enabled.get("SURF", False) and detectors.get("SURF") is not None:
        tags.append("SUP")
    return tags


def make_unique_path(path: str) -> str:
    """
    If 'path' exists, append _2, _3, ... before the extension.
    """
    if not os.path.exists(path):
        return path
    root, ext = os.path.splitext(path)
    i = 2
    while True:
        candidate = f"{root}_{i}{ext}"
        if not os.path.exists(candidate):
            return candidate
        i += 1

    
def main():
    ap = argparse.ArgumentParser(description="Two-pane frame matcher demo (ORB/SIFT/SURF overlays).")
    ap.add_argument("frames_dir", help="Directory containing frames/images")
    ap.add_argument("--scale", type=float, default=1.0, help="Display scale (e.g. 0.5)")
    ap.add_argument("--ratio", type=float, default=0.75, help="Lowe ratio test threshold (default 0.75)")
    ap.add_argument("--max-matches", type=int, default=300, help="Max matches per algorithm to draw")
    ap.add_argument("--orb-features", type=int, default=2000, help="ORB nfeatures")
    ap.add_argument("--surf-hessian", type=float, default=400.0, help="SURF hessian threshold (if available)")
    args = ap.parse_args()

    paths = list_images(args.frames_dir)
    if not paths:
        raise SystemExit(f"No images found in: {args.frames_dir}")

    # Detectors (graceful if unavailable)
    detectors = {
        "ORB": create_orb(args.orb_features),
        "SIFT": try_create_sift(),
        "SURF": try_create_surf(args.surf_hessian),
    }

    # Colors are BGR
    algo_specs = {
        "ORB": AlgoSpec("ORB", (0, 255, 255)),   # yellow
        "SIFT": AlgoSpec("SIFT", (0, 255, 0)),   # green
        "SURF": AlgoSpec("SURF", (255, 0, 0)),   # blue
    }

    available = {k: (v is not None) for k, v in detectors.items()}
    enabled = {
        "ORB": True,
        "SIFT": available["SIFT"],
        "SURF": False,  # default off; switch on if you have it
    }

    # Cache: cache[(algo, path)] = (kps, desc)
    cache: Dict[Tuple[str, str], Tuple[List[cv2.KeyPoint], Optional[np.ndarray]]] = {}

    left_idx = 0
    right_idx = min(1, len(paths) - 1)
    active_side = "Left"
    show_help = True

    geom_modes = ["OFF", "H", "F"]   # off, homography, fundamental
    geom_mode_i = 1                 # start with homography (set 0 to start OFF)
    ransac_thresh_px = 3.0

    
    win = "Frame-to-frame matching (Left | Right)"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)

    output_dir = os.path.join(args.frames_dir, "output")
    os.makedirs(output_dir, exist_ok=True)
    
    def load_scaled(idx: int) -> Tuple[np.ndarray, str]:
        p = paths[idx]
        img = cv2.imread(p, cv2.IMREAD_COLOR)
        if img is None:
            raise RuntimeError(f"Failed to read: {p}")
        img = resize_keep_aspect(img, args.scale)
        name = os.path.basename(p)
        return img, name

    def get_features(algo: str, path: str, img_gray: np.ndarray):
        key = (algo, path)
        if key in cache:
            return cache[key]
        det = detectors[algo]
        if det is None:
            cache[key] = ([], None)
            return cache[key]
        kps, desc = compute_kp_desc(det, img_gray)
        cache[key] = (kps, desc)
        return cache[key]

    def recompute_and_render() -> np.ndarray:
        left_img, left_name = load_scaled(left_idx)
        right_img, right_name = load_scaled(right_idx)

        left_gray = to_gray(left_img)
        right_gray = to_gray(right_img)

        results = {}
        counts = {}

        for algo, on in enabled.items():
            if not on:
                continue
            if detectors[algo] is None:
                continue

            p1 = paths[left_idx]
            p2 = paths[right_idx]

            kps1, desc1 = get_features(algo, p1, left_gray)
            kps2, desc2 = get_features(algo, p2, right_gray)

            matches = match_desc(algo, desc1, desc2, ratio_test=args.ratio)
            if len(matches) > args.max_matches:
                matches = matches[: args.max_matches]

            geom_mode = geom_modes[geom_mode_i]

            try:
            
                if geom_mode != "OFF":
                    if geom_mode == "H":
                        inliers, model, _ = ransac_filter_matches_homography(
                            kps1, kps2, matches, ransac_thresh_px
                        )
                        matches = inliers
                    elif geom_mode == "F":
                        inliers, model, _ = ransac_filter_matches_fundamental(
                            kps1, kps2, matches, ransac_thresh_px
                        )
                        matches = inliers

            except:
                matches = []

            results[algo] = {"kps1": kps1, "kps2": kps2, "matches": matches}
            counts[algo] = (len(kps1), len(kps2), len(matches))

        canvas = draw_kps_and_matches(left_img, right_img, results, algo_specs)


        # Warn if user enabled algo that isn't available
        y = 110
        for algo in ["SIFT", "SURF"]:
            if enabled.get(algo, False) and detectors[algo] is None:
                cv2.putText(
                    canvas,
                    f"{algo} not available in this OpenCV build",
                    (10, y),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 0, 255),
                    2,
                    cv2.LINE_AA,
                )
                y += 26

        geom_mode = geom_modes[geom_mode_i]

        put_hud(
            canvas,
            left_name,
            right_name,
            left_idx,
            right_idx,
            len(paths),
            enabled,
            active_side,
            counts,
            geom_mode,
            ransac_thresh_px,
            show_help
        )        
        # status = f"Geom: {geom_mode}  thr={ransac_thresh_px:.1f}px"
        # cv2.putText(canvas, status, (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 0), 3, cv2.LINE_AA)
        # cv2.putText(canvas, status, (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (240, 240, 240), 1, cv2.LINE_AA)
        
        return canvas

    dirty = True

    # Common extended keycodes (these vary, so we also provide fallbacks)
    ARROW_LEFT = {2424832, 65361}
    ARROW_RIGHT = {2555904, 65363}
    ARROW_UP = {2490368, 65362}
    ARROW_DOWN = {2621440, 65364}

    last_frame = None
    
    while True:
        if dirty or last_frame is None:
            last_frame = recompute_and_render()
            dirty = False

        imshow_resized(win, last_frame)

        k = cv2.waitKeyEx(30)
        if k == 27:  # ESC
            break

        # Toggle help
        if key_is(k, "hH"):
            show_help = not show_help
            dirty = True
            continue

        # Switch active side
        if k == 9:  # Tab
            active_side = "Right" if active_side == "Left" else "Left"
            dirty = True
            continue

        # Reset toggles
        if key_is(k, "rR"):
            enabled["ORB"] = True
            enabled["SIFT"] = available["SIFT"]
            enabled["SURF"] = False
            dirty = True
            geom_mode_i = 1
            ransac_thresh_px = 3.0
            continue

        # Toggle algos
        if key_is(k, "1"):
            enabled["ORB"] = not enabled["ORB"]
            dirty = True
            continue
        if key_is(k, "2"):
            enabled["SIFT"] = (not enabled["SIFT"]) if available["SIFT"] else False
            dirty = True
            continue
        if key_is(k, "3"):
            enabled["SURF"] = (not enabled["SURF"]) if available["SURF"] else False
            dirty = True
            continue

        # Cycle geometric verification mode: OFF -> H -> F -> OFF
        if key_is(k, "gG"):
            geom_mode_i = (geom_mode_i + 1) % len(geom_modes)
            print(f"[geom] mode = {geom_modes[geom_mode_i]}")
            dirty = True
            continue

        # Adjust geometric threshold (pixels)
        if key_is(k, "["):
            ransac_thresh_px = max(0.5, ransac_thresh_px - 0.5)
            print(f"[geom] thresh = {ransac_thresh_px:.1f}px")
            dirty = True
            continue
        if key_is(k, "]"):
            ransac_thresh_px = min(50.0, ransac_thresh_px + 0.5)
            print(f"[geom] thresh = {ransac_thresh_px:.1f}px")
            dirty = True
            continue

        # Recompute
        if key_is(k, "mM"):
            dirty = True
            continue

        # Save current frame
        if key_is(k, "sS"):
            left_stem = os.path.splitext(os.path.basename(paths[left_idx]))[0]
            right_stem = os.path.splitext(os.path.basename(paths[right_idx]))[0]
            tags = build_active_tags(enabled, detectors)

            # Base name: imgfile1_imgfile2_SIFT_ORB_SUP.png (tags optional)
            parts = [left_stem, right_stem] + tags
            out_name = "_".join(parts) + ".png"
            out_path = make_unique_path(os.path.join(output_dir, out_name))

            ok = cv2.imwrite(out_path, last_frame)
            if ok:
                print(f"[saved] {out_path}")
            else:
                print(f"[save failed] {out_path}")

            continue        

        def step(which: str, delta: int):
            nonlocal left_idx, right_idx
            if which == "Left":
                left_idx = (left_idx + delta) % len(paths)
            else:
                right_idx = (right_idx + delta) % len(paths)

        # Dedicated keys (reliable across platforms)
        if key_is(k, "aA"):
            step("Left", -1); dirty = True; continue
        if key_is(k, "dD"):
            step("Left", +1); dirty = True; continue
        if key_is(k, "jJ"):
            step("Right", -1); dirty = True; continue
        if key_is(k, "lL"):
            step("Right", +1); dirty = True; continue

        # Arrow keys (best-effort, depends on OS/backend)
        if k in ARROW_LEFT:
            step(active_side, -1); dirty = True; continue
        if k in ARROW_RIGHT:
            step(active_side, +1); dirty = True; continue
        if k in ARROW_UP or k in ARROW_DOWN:
            active_side = "Right" if active_side == "Left" else "Left"
            dirty = True
            continue

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
