import numpy as np

def overshoot(response, target):
    response = np.asarray(response)
    max_val = np.max(response)
    return max(0.0, (max_val - target) / target * 100.0) if target != 0 else 0.0

def settling_time(t, response, target, tol=0.02):
    """
    Settling time: time after which response stays within +/- tol*target.
    tol = 0.02 → 2%
    """
    response = np.asarray(response)
    band_low = target * (1 - tol)
    band_high = target * (1 + tol)

    inside = np.logical_and(response >= band_low, response <= band_high)
    # Find first time after which it stays inside
    for i in range(len(response)):
        if np.all(inside[i:]):
            return t[i]
    return t[-1]
