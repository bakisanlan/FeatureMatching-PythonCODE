import numpy as np

def zncc_batch_patches(patches, template, eps=1e-12):
    """
    patches: (N,h,w) float/uint8
    template: (h,w)
    returns: (N,) ZNCC scores
    """
    P = patches.astype(np.float32).reshape(patches.shape[0], -1)  # (N, hw)
    T = template.astype(np.float32).ravel()                        # (hw,)

    T0 = T - T.mean()
    denom_T = np.sqrt(np.dot(T0, T0)) + eps

    Pm = P.mean(axis=1, keepdims=True)
    P0 = P - Pm

    num = P0 @ T0                          # (N,)
    denom = (np.linalg.norm(P0, axis=1) * denom_T) + eps

    return num / denom


def zncc_batch_patches(patches: np.ndarray, template: np.ndarray, eps: float = 1e-12) -> np.ndarray:
    """Unmasked ZNCC (baseline). patches: (N,h,w), template: (h,w) -> (N,)"""
    P = patches.astype(np.float32).reshape(patches.shape[0], -1)
    T = template.astype(np.float32).ravel()

    T0 = T - T.mean()
    denom_T = np.sqrt(np.dot(T0, T0)) + eps

    P0 = P - P.mean(axis=1, keepdims=True)
    num = P0 @ T0
    denom = (np.linalg.norm(P0, axis=1) * denom_T) + eps
    return num / denom


def masked_zncc_batch_patches(patches: np.ndarray,
                             template: np.ndarray,
                             mask: np.ndarray,
                             eps: float = 1e-12) -> np.ndarray:
    """
    Masked ZNCC (ignores mask==0 pixels).
    patches : (N,h,w) satellite patches (particles)
    template: (h,w) UAV template (orthoprojected)
    mask    : (h,w) 1=valid, 0=invalid (black region)
    returns : (N,) masked ZNCC scores
    """
    P = patches.astype(np.float32).reshape(patches.shape[0], -1)   # (N,M)
    T = template.astype(np.float32).ravel()                        # (M,)
    M = mask.astype(np.float32).ravel()                            # (M,)

    w = M.sum()
    if w < 1:
        raise ValueError("Mask has no valid pixels (sum(mask)==0).")

    w = w + eps

    # Weighted (masked) zero-mean template
    mu_T = (M * T).sum() / w
    T0 = (T - mu_T) * M
    denom_T = np.sqrt((T0 * T0).sum()) + eps

    # Per-patch weighted mean/variance over valid pixels only
    S  = (P * M).sum(axis=1)              # sum M*I
    S2 = ((P * P) * M).sum(axis=1)        # sum M*I^2
    var = S2 - (S * S) / w                # sum M*(I - mu_I)^2
    var = np.maximum(var, eps)
    denom_I = np.sqrt(var)

    # Numerator becomes a dot product / correlation with T0
    num = P @ T0

    return num / (denom_I * denom_T)


def extract_patches(img: np.ndarray, xs: np.ndarray, ys: np.ndarray, h: int, w: int) -> np.ndarray:
    """Simple patch extraction (loop). Scoring remains fully vectorized."""
    patches = np.empty((len(xs), h, w), dtype=img.dtype)
    for i, (x, y) in enumerate(zip(xs, ys)):
        patches[i] = img[y:y+h, x:x+w]
    return patches


if __name__ == "__main__":
    rng = np.random.default_rng(0)

    # --- Load UAV orthoprojection (grayscale) ---
    # Replace path with your own UAV image
    uav = cv2.imread("/home/ituarc/Desktop/uav_correct.png", cv2.IMREAD_GRAYSCALE)
    if uav is None:
        raise FileNotFoundError("Could not read UAV image. Check the path.")

    # Downsample so the demo runs fast
    uav = cv2.resize(uav, (300, 300), interpolation=cv2.INTER_AREA)  # (w,h) in OpenCV
    template = uav
    h, w = template.shape

    # --- Build a validity mask ---
    # Quick demo mask: invalid pixels are (near) black from orthoprojection
    # In production, best practice is to warp an all-ones mask with the same transform.
    mask = (template > 5).astype(np.uint8)

    # --- Create a synthetic satellite image and embed the UAV template once ---
    H, W = 2000, 2000
    sat = rng.integers(0, 256, size=(H, W), dtype=np.uint8)

    true_x, true_y = 180, 130

    # Embed a “matching” patch but with illumination change + noise
    embedded = template.astype(np.float32) * 1.15 + 18.0 + rng.normal(0, 6.0, size=template.shape)
    embedded = np.clip(embedded, 0, 255).astype(np.uint8)
    sat[true_y:true_y+h, true_x:true_x+w] = embedded

    # --- Particle hypotheses (top-left coords) ---
    N = 200
    xs = rng.integers(0, W - w + 1, size=N, dtype=np.int32)
    ys = rng.integers(0, H - h + 1, size=N, dtype=np.int32)
    xs[0], ys[0] = true_x, true_y  # ensure truth is included

    # --- Extract patches & score ---
    patches = extract_patches(sat, xs, ys, h, w)

    scores_masked = masked_zncc_batch_patches(patches, template, mask)
    scores_plain  = zncc_batch_patches(patches, template)

    best_m = int(np.argmax(scores_masked))
    best_p = int(np.argmax(scores_plain))

    print("=== Masked ZNCC ===")
    print(f"best idx: {best_m}, best (x,y)=({xs[best_m]}, {ys[best_m]}), score={scores_masked[best_m]:.4f}")
    print(f"true (x,y)=({true_x}, {true_y}), score={scores_masked[0]:.4f}")

    print("\n=== Plain (unmasked) ZNCC ===")
    print(f"best idx: {best_p}, best (x,y)=({xs[best_p]}, {ys[best_p]}), score={scores_plain[best_p]:.4f}")
    print(f"true (x,y)=({true_x}, {true_y}), score={scores_plain[0]:.4f}")
