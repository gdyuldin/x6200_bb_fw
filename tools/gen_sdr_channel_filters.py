#!/usr/bin/env python3
"""
Generate SDR channel FIR coefficients for the SSB channel filter.

The filter is a complex band-pass built by modulating a real low-pass
prototype by +shift, with the shift fixed at fs/4. Because
exp(j*2*pi*(fs/4)*n/fs) = j^n, the resulting coefficients alternate: even
taps are purely real, odd taps purely imaginary. Only the non-zero half of
each part is stored, and the filter is applied so that every tap costs one
multiply instead of two.

Configuration:

  * f_pass_left_hz  - left passband edge of the complex filter;
  * f_stop_left_hz  - left stopband edge;
  * shift_div       - shift = fs / shift_div, must be 4;
  * order           - prototype order N (num_taps = N + 1);
  * weight_pass     - Remez passband weight;
  * weight_stop     - Remez stopband weight.

With shift fixed, the prototype band edges follow from the left side only:

    f_p = shift - f_pass_left
    f_s = shift - f_stop_left

The Remez prototype is designed on [0, f_p, f_s, fs/2] with weights
[weight_pass, weight_stop]. The shifted passband is necessarily symmetric
about shift, so the upper edge cannot be chosen independently; the upper
stop edge may fall beyond fs/2 and is then truncated by Nyquist.

Coefficients are normalized by COMPLEX_GAIN and stored compressed, in natural
order:

    sdr_ssb_coeffs_re[m] = h_re[2*m]      (even taps)
    sdr_ssb_coeffs_im[m] = h_im[2*m + 1]  (odd taps)

In the DSP they are applied as single-multiply-per-tap MACs:

    RX: y[n] = sum re[m]*i[n-2m] - sum im[m]*q[n-2m-1]   (USB; '+' for LSB)
    TX: i[n] = sum re[m]*a[n-2m], q[n] = sum im[m]*a[n-2m-1]
"""

from pathlib import Path

import numpy as np
import scipy.signal as sig

OUTPUT_PATH = Path(__file__).resolve().parent.parent / "App/SDR/sdr_channel_filters.h"

COMPLEX_GAIN = 2.0
REMEZ_MAXITER = 100

FILTERS = [
    {
        # 200 Hz left transition, fixed fs/4 shift, 12 kHz rate
        "name": "sdr_ssb",
        "kind": "complex_bandpass",
        "fs_hz": 12000,
        "shift_div": 4,
        "f_pass_left_hz": 150,
        "f_stop_left_hz": -50,
        "order": 248,
        "weight_pass": 1.0,
        "weight_stop": 28.8,
        "atten_db": 80.0,
        "note": "SSB complex band-pass, shift = fs/4 (alternating re/im taps)",
    },
]


def complex_band_params(cfg: dict) -> tuple[float, float, float]:
    """Return (shift_hz, f_p_hz, f_s_hz) for the Remez low-pass prototype."""
    fs = float(cfg["fs_hz"])
    shift_div = int(cfg.get("shift_div", 4))
    if shift_div != 4:
        raise ValueError(
            f"{cfg['name']}: shift_div must be 4 for alternating re/im taps"
        )

    shift = fs / float(shift_div)
    f_stop_left = float(cfg["f_stop_left_hz"])
    f_pass_left = float(cfg["f_pass_left_hz"])

    f_p = shift - f_pass_left
    f_s = shift - f_stop_left

    if not (f_stop_left < f_pass_left < shift):
        raise ValueError(
            f"{cfg['name']}: require f_stop_left_hz < f_pass_left_hz < shift "
            f"({shift:g} Hz)"
        )
    if not (0.0 < f_p < f_s <= 0.5 * fs):
        raise ValueError(
            f"{cfg['name']}: invalid prototype band edges "
            f"(f_p={f_p:g} Hz, f_s={f_s:g} Hz, fs/2={0.5 * fs:g} Hz)"
        )

    return shift, f_p, f_s


def format_float(x: float) -> str:
    s = f"{float(x):.9g}"
    if "." not in s and "e" not in s and "E" not in s:
        s += ".0"
    return s + "f"


def format_array(values: np.ndarray, indent: str = "    ") -> str:
    return ",\n".join(indent + format_float(v) for v in values) + ","


def find_band_edges(
    freq: np.ndarray,
    mag: np.ndarray,
    level_db: float,
    peak: float,
) -> tuple[float, float]:
    threshold = peak * 10.0 ** (level_db / 20.0)
    above = mag >= threshold
    if not np.any(above):
        return float("nan"), float("nan")

    # Find the contiguous band containing the peak. This avoids reporting a
    # wide span when the stopband has equiripple points exactly at the level.
    peak_idx = int(np.argmax(mag))
    first = peak_idx
    while first > 0 and above[first - 1]:
        first -= 1
    last = peak_idx
    while last < mag.size - 1 and above[last + 1]:
        last += 1

    if first == 0:
        lower = float(freq[0])
    else:
        lower = float(
            freq[first - 1]
            + (threshold - mag[first - 1])
            * (freq[first] - freq[first - 1])
            / (mag[first] - mag[first - 1])
        )

    if last == freq.size - 1:
        upper = float(freq[-1])
    else:
        upper = float(
            freq[last]
            + (threshold - mag[last])
            * (freq[last + 1] - freq[last])
            / (mag[last + 1] - mag[last])
        )

    return lower, upper


def measure_complex(cfg: dict, b: np.ndarray) -> dict:
    fs = float(cfg["fs_hz"])
    w, h = sig.freqz(b, 1.0, worN=262144, whole=True, fs=fs)
    w = w - 0.5 * fs
    h = np.fft.fftshift(h)
    mag = np.abs(h)
    peak = float(np.max(mag))

    atten_db = float(cfg.get("atten_db", 80.0))
    f6_lo, f6_hi = find_band_edges(w, mag, -6.0, peak)
    f80_lo, f80_hi = find_band_edges(w, mag, -atten_db, peak)

    shift, f_p, f_s = complex_band_params(cfg)
    pass_lo = shift - f_p
    pass_hi = shift + f_p
    pass_mask = (w >= pass_lo) & (w <= pass_hi)
    if np.any(pass_mask):
        pass_max = float(np.max(mag[pass_mask]))
        pass_min = float(np.min(mag[pass_mask]))
        pass_ripple_db = 20.0 * np.log10(pass_max / pass_min)
    else:
        pass_ripple_db = float("nan")

    dc_level = float(np.interp(0.0, w, mag))
    dc_atten_db = -20.0 * np.log10(dc_level / peak)

    # Negative frequencies that fold into the used passband when the real part
    # of the complex output is taken.
    mirror_mask = (w >= -pass_hi) & (w <= -pass_lo)
    if np.any(mirror_mask):
        mirror_max = float(np.max(mag[mirror_mask]))
        mirror_atten_db = -20.0 * np.log10(mirror_max / peak)
    else:
        mirror_atten_db = float("inf")

    # Upper region above the passband; may be truncated by fs/2 when the
    # theoretical stop edge lies beyond Nyquist.
    upper_mask = (w >= pass_hi) & (w <= 0.5 * fs)
    if np.any(upper_mask):
        upper_max = float(np.max(mag[upper_mask]))
        upper_atten_db = -20.0 * np.log10(upper_max / peak)
    else:
        upper_atten_db = float("inf")

    return {
        "f6_lo_hz": f6_lo,
        "f6_hi_hz": f6_hi,
        "f80_lo_hz": f80_lo,
        "f80_hi_hz": f80_hi,
        "pass_lo_hz": pass_lo,
        "pass_hi_hz": pass_hi,
        "f_p_hz": f_p,
        "f_s_hz": f_s,
        "shift_hz": shift,
        "pass_ripple_db": pass_ripple_db,
        "dc_atten_db": dc_atten_db,
        "mirror_atten_db": mirror_atten_db,
        "upper_atten_db": upper_atten_db,
        "stop_left_hz": float(cfg["f_stop_left_hz"]),
    }


def design_complex_remez(cfg: dict) -> dict:
    fs = float(cfg["fs_hz"])
    shift, f_p, f_s = complex_band_params(cfg)

    order = int(cfg["order"])
    num_taps = order + 1
    if num_taps < 2:
        raise ValueError(f"{cfg['name']}: order must be at least 1")

    weight_pass = float(cfg["weight_pass"])
    weight_stop = float(cfg["weight_stop"])

    try:
        b = sig.remez(
            num_taps,
            [0.0, f_p, f_s, 0.5 * fs],
            [1.0, 0.0],
            weight=[weight_pass, weight_stop],
            fs=fs,
            maxiter=REMEZ_MAXITER,
        )
    except Exception as exc:
        raise ValueError(f"{cfg['name']}: remez design failed: {exc}") from exc

    n = np.arange(num_taps)
    h = b * np.exp(1j * 2.0 * np.pi * shift * n / fs) * COMPLEX_GAIN

    coeffs_re = h.real.astype(np.float32)
    coeffs_im = h.imag.astype(np.float32)

    # The modulation factor is exactly j^n; zero the nominally-zero half so
    # the alternation is exact and only the non-zero halves are needed.
    coeffs_re[1::2] = 0.0
    coeffs_im[0::2] = 0.0

    hq = coeffs_re.astype(np.float64) + 1j * coeffs_im.astype(np.float64)
    metrics = measure_complex(cfg, hq)

    coeffs_re_even = coeffs_re[0::2].copy()
    coeffs_im_odd = coeffs_im[1::2].copy()

    return {
        "cfg": cfg,
        "method": (
            f"remez-complex (N={order}, "
            f"weight_pass={weight_pass:g}, weight_stop={weight_stop:g})"
        ),
        "num_taps": num_taps,
        "order": order,
        "even_taps": int(coeffs_re_even.size),
        "odd_taps": int(coeffs_im_odd.size),
        "coeffs_re": coeffs_re_even,
        "coeffs_im": coeffs_im_odd,
        "fc_hz": f_p,
        "shift_hz": shift,
        "f_p_hz": f_p,
        "f_s_hz": f_s,
        "weight_pass": weight_pass,
        "weight_stop": weight_stop,
        "metrics": metrics,
    }


def design(cfg: dict) -> dict:
    if cfg["kind"] != "complex_bandpass":
        raise ValueError(f"{cfg['name']}: unsupported filter kind {cfg['kind']!r}")
    return design_complex_remez(cfg)


def build_header(results: list[dict]) -> str:
    lines = [
        "#pragma once",
        "",
        "#include <arm_math.h>",
        "",
        "/*",
        " * SDR channel FIR coefficients for the SSB channel filter.",
        " * Generated by tools/gen_sdr_channel_filters.py. Do not edit by hand.",
        " *",
        " * The shift is fixed at fs/4, so coefficients alternate: even taps are",
        " * purely real, odd taps purely imaginary. Only the non-zero halves are",
        " * stored, in natural order, and each tap costs one multiply.",
        " */",
        "",
    ]

    for res in results:
        cfg = res["cfg"]
        name = cfg["name"]
        macro = name.upper() + "_NUM_TAPS"
        even_macro = name.upper() + "_EVEN_TAPS"
        odd_macro = name.upper() + "_ODD_TAPS"
        num_taps = res["num_taps"]
        metrics = res["metrics"]
        delay_samples = 0.5 * (num_taps - 1)
        delay_ms = 1000.0 * delay_samples / cfg["fs_hz"]
        shift, f_p, f_s = complex_band_params(cfg)
        pass_lo = shift - f_p
        pass_hi = shift + f_p
        atten_db = float(cfg.get("atten_db", 80.0))

        lines.extend([
            "/*",
            f" * {name}",
            f" *   Kind            : complex band-pass FIR, shift fixed at fs/4",
            f" *   Input rate      : {cfg['fs_hz']} Hz",
            f" *   Shift           : +{shift:g} Hz (fs/{int(cfg.get('shift_div', 4))})",
            f" *   Passband        : {pass_lo:g} .. {pass_hi:g} Hz",
            f" *   Lower transition: {cfg['f_stop_left_hz']:g} .. {pass_lo:g} Hz",
            f" *   Atten reference : {atten_db:g} dB (reporting only)",
            f" *   Method          : {res['method']}",
            f" *   Prototype       : remez band edges {f_p:g} .. "
            f"{f_s:g} Hz (low-pass)",
            f" *   Weights         : pass={res['weight_pass']:g}, "
            f"stop={res['weight_stop']:g}",
            f" *   Order           : N={res['order']} ({num_taps} taps: "
            f"{res['even_taps']} real + {res['odd_taps']} imag)",
            f" *   Measured -6 dB  : {metrics['f6_lo_hz']:.1f} .. "
            f"{metrics['f6_hi_hz']:.1f} Hz",
            f" *   Measured -{atten_db:g} dB : {metrics['f80_lo_hz']:.1f} .. "
            f"{metrics['f80_hi_hz']:.1f} Hz",
            f" *   Passband ripple : {metrics['pass_ripple_db']:.3f} dB",
            f" *   DC rejection    : {metrics['dc_atten_db']:.1f} dB at 0 Hz",
            f" *   Mirror rejection: {metrics['mirror_atten_db']:.1f} dB "
            f"(negative freqs folding into the passband)",
            f" *   Upper rejection : {metrics['upper_atten_db']:.1f} dB "
            f"(>= {pass_hi:g} Hz, truncated by Nyquist; IIR removes it)",
            f" *   Group delay     : {delay_samples:g} samples ({delay_ms:.3f} ms)",
            f" *   Gain            : coefficients x{COMPLEX_GAIN:g}",
            f" *   Purpose         : {cfg['note']}",
            " *",
            " *   Stored compressed in natural order:",
            f" *     {name}_coeffs_re[m] = h_re[2*m]       m = 0 .. {even_macro}-1",
            f" *     {name}_coeffs_im[m] = h_im[2*m+1]     m = 0 .. {odd_macro}-1",
            " *",
            " *   RX: y[n] = sum re[m]*i[n-2m] - sum im[m]*q[n-2m-1]",
            " *       (LSB uses '+' on the second sum).",
            " *   TX: i[n] = sum re[m]*a[n-2m], q[n] = sum im[m]*a[n-2m-1];",
            " *       conjugate q (negate) for LSB.",
            " */",
            f"#define {macro} {num_taps}u",
            f"#define {even_macro} {res['even_taps']}u",
            f"#define {odd_macro} {res['odd_taps']}u",
            "",
            f"static const float32_t {name}_coeffs_re[{even_macro}] = {{",
            format_array(res["coeffs_re"]),
            "};",
            "",
            f"static const float32_t {name}_coeffs_im[{odd_macro}] = {{",
            format_array(res["coeffs_im"]),
            "};",
            "",
        ])

    return "\n".join(lines)


def report_complex(cfg: dict, res: dict) -> None:
    metrics = res["metrics"]
    delay_samples = 0.5 * (res["num_taps"] - 1)
    delay_ms = 1000.0 * delay_samples / cfg["fs_hz"]
    shift, f_p, f_s = complex_band_params(cfg)
    pass_lo = shift - f_p
    pass_hi = shift + f_p
    atten_db = float(cfg.get("atten_db", 80.0))

    print(f"{cfg['name']}: {cfg['fs_hz']} Hz, {res['method']}")
    print(f"  shift    : +{shift:g} Hz (fs/{int(cfg.get('shift_div', 4))})")
    print(f"  passband : {pass_lo:g} .. {pass_hi:g} Hz")
    print(
        f"  lower transition: {cfg['f_stop_left_hz']:g} .. {pass_lo:g} Hz"
    )
    print(f"  order N  : {res['order']} ({res['num_taps']} taps: "
          f"{res['even_taps']} real + {res['odd_taps']} imag)")
    print(f"  weights  : pass={res['weight_pass']:g}, stop={res['weight_stop']:g}")
    print(f"  -6 dB    : {metrics['f6_lo_hz']:.1f} .. {metrics['f6_hi_hz']:.1f} Hz")
    print(
        f"  -{atten_db:g} dB   : {metrics['f80_lo_hz']:.1f} .. "
        f"{metrics['f80_hi_hz']:.1f} Hz"
    )
    print(f"  passband ripple : {metrics['pass_ripple_db']:.3f} dB")
    print(f"  DC       : {metrics['dc_atten_db']:.1f} dB (at 0 Hz)")
    print(f"  mirror   : {metrics['mirror_atten_db']:.1f} dB (negative freqs)")
    print(f"  upper    : {metrics['upper_atten_db']:.1f} dB "
          f"(>= {pass_hi:g} Hz, truncated)")
    print(f"  delay    : {delay_samples:g} samples ({delay_ms:.3f} ms)")


def main() -> int:
    results = []
    for cfg in FILTERS:
        try:
            res = design(cfg)
        except ValueError as exc:
            print(f"ERROR: {exc}")
            return 1

        results.append(res)
        report_complex(cfg, res)
        print()

    header = build_header(results)
    OUTPUT_PATH.parent.mkdir(parents=True, exist_ok=True)
    OUTPUT_PATH.write_text(header, encoding="utf-8")
    print(f"Wrote {OUTPUT_PATH}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())