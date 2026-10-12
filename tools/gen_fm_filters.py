#!/usr/bin/env python3
"""
Generate FM filter coefficients for the x6200 SDR firmware.

One script, two profiles selected with --profile {nbfm,wfm}. It writes a C
header (default per profile, see PROFILES) with:

  * <SYMBOL>_FIR_NUM_TAPS and <base>_fir_coeffs[]  (FIR differentiator)
  * <SYMBOL>_PRE_IIR_INIT and <SYMBOL>_DE_IIR_INIT (first-order IIR
    pre/de-emphasis compatible with the FirstOrderIIR prototype)
  * one FIR low-pass per entry in the profile's LPF table.

IIR coefficients come from the bilinear transform of the analog prototype

    H(s) = (s + wc) / (s + w_high)

with wc = 2*pi*f_corner and w_high = 2*pi*iir_f_high. Both filters are
normalized to 0 dB at the reference frequency f_ref (default 1 kHz), so the
pre-emphasis and de-emphasis responses cross unity gain there. The de-emphasis
coefficients are swapped from the normalized pre-emphasis and renormalized so
that a0 = 1. This matches the provided first_order_iir() difference equation,
which assumes a0 = 1, and makes the pre/de-emphasis cascade exactly flat.

FIR coefficients are designed with scipy.signal.remez: a type='differentiator'
low-pass differentiator for the discriminator, and a type='bandpass' 0 dB
passband low-pass for each LPF-table entry.

The IIR is designed at --iir-fs (default = --fs). This is used when the
discriminator runs at a higher rate than the de-emphasis stage (WFM: the
differentiator runs at 192 kHz, de-emphasis after decimation at 48 kHz).

Examples:

    python3 tools/gen_fm_filters.py --profile nbfm
    python3 tools/gen_fm_filters.py --profile wfm
    python3 tools/gen_fm_filters.py --profile nbfm --f-corner 300 --plot

Plot generation is optional and requires matplotlib.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np
from scipy import signal

TOOLS_DIR = Path(__file__).resolve().parent
ROOT_DIR = TOOLS_DIR.parent


def _defaults(**overrides: float) -> dict[str, float]:
    base = {
        "fs": 48000.0,
        "f_corner": 300.0,
        "iir_f_high": 4000.0,
        "f_ref": 1000.0,
        "f_pass": 4000.0,
        "f_stop": 6500.0,
        "fir_taps": 40,
        "fir_weight_pass": 1.0,
        "fir_weight_stop": 10.0,
        "fir_slope": 1.0,
        "fir_gain": 1.0,
    }
    base.update(overrides)
    return base


NBFM_LPF_FILTERS = (
    {
        "name": "1k",
        "f_pass_khz": 4.0,
        "f_stop_khz": 5.0,
        "order": 118,
        "weight_pass": 1.0,
        "weight_stop": 23.0,
    },
    {
        "name": "2k5",
        "f_pass_khz": 5.5,
        "f_stop_khz": 6.5,
        "order": 118,
        "weight_pass": 1.0,
        "weight_stop": 23.0,
    },
    {
        "name": "5k",
        "f_pass_khz": 8.0,
        "f_stop_khz": 9.0,
        "order": 118,
        "weight_pass": 1.0,
        "weight_stop": 23.0,
    },
)

PROFILES = {
    "nbfm": {
        "brief": "NBFM",
        "headline": "NBFM (narrowband FM) filter coefficients.",
        "symbol": "SDR_NBFM",
        "base": "sdr_nbfm",
        "lpf_filters": NBFM_LPF_FILTERS,
        "defaults": _defaults(),
        "output": ROOT_DIR / "App/SDR/sdr_nbfm_filters.h",
        "plot": TOOLS_DIR / "nbfm_filters_response.png",
        "plot_title_fir": "NBFM FIR differentiator",
        "plot_title_iir": "NBFM IIR pre/de-emphasis (0 dB at f_ref)",
    },
    "wfm": {
        "brief": "WFM",
        "headline": "WFM (wideband FM, mono) filter coefficients.",
        "symbol": "SDR_WFM",
        "base": "sdr_wfm",
        # No post-discriminator audio LPF table: the 192 -> 48 kHz decimation
        # stages already band-limit the audio to ~15 kHz.
        "lpf_filters": (),
        "defaults": _defaults(
            fs=192000.0,
            f_corner=2122.0,   # 75 us broadcast de-emphasis pole
            iir_f_high=15000.0,
            f_ref=1000.0,
            f_pass=15000.0,
            f_stop=20000.0,
            fir_taps=40,
            fir_weight_stop=2,
        ),
        "output": ROOT_DIR / "App/SDR/sdr_wfm_filters.h",
        "plot": TOOLS_DIR / "wfm_filters_response.png",
        "plot_title_fir": "WFM FIR differentiator",
        "plot_title_iir": "WFM IIR pre/de-emphasis (0 dB at f_ref)",
    },
}


def format_float(value: float) -> str:
    """Format a float as a C float literal."""
    text = f"{float(value):.9g}"
    if "." not in text and "e" not in text and "E" not in text:
        text += ".0"
    return text + "f"


def format_array(values: np.ndarray, indent: str = "    ") -> str:
    """Format a numpy array as a C initializer list."""
    return ",\n".join(indent + format_float(v) for v in values) + ","


def filter_c_identifier(name: str) -> str:
    """Convert a filter name into a lowercase C identifier fragment."""
    identifier = "".join(ch if ch.isalnum() else "_" for ch in name.lower())
    return identifier or "unnamed"


def lpf_num_taps(order: int) -> int:
    """Return the smallest multiple of 4 that is at least ``order + 1``."""
    minimum = int(order) + 1
    return ((minimum + 3) // 4) * 4


def design_fir_lpf(
    fs_hz: float,
    taps: int,
    f_pass_hz: float,
    f_stop_hz: float,
    weight_pass: float,
    weight_stop: float,
) -> np.ndarray:
    """Design a 0 dB passband FIR low-pass filter with Remez exchange.

    The passband is ``0 .. f_pass_hz`` and the stopband is
    ``f_stop_hz .. Fs/2``.  No post-design normalization is applied, so the
    passband stays at approximately 0 dB as requested.
    """
    bands = np.array([0.0, f_pass_hz, f_stop_hz, 0.5 * fs_hz], dtype=float)
    coefficients = signal.remez(
        taps,
        bands,
        [1.0, 0.0],
        weight=[weight_pass, weight_stop],
        type="bandpass",
        fs=fs_hz,
    )
    return np.asarray(coefficients, dtype=float)


def measure_fir_lpf(
    coefficients: np.ndarray,
    fs_hz: float,
    f_pass_hz: float,
    f_stop_hz: float,
) -> dict[str, float]:
    """Measure passband ripple and stopband attenuation in dB."""
    frequency, response = signal.freqz(
        coefficients, worN=1 << 16, fs=fs_hz
    )
    magnitude = np.abs(response)

    passband = magnitude[frequency <= f_pass_hz]
    stopband = magnitude[frequency >= f_stop_hz]
    passband_db = 20.0 * np.log10(passband + 1e-12)
    stopband_db = 20.0 * np.log10(stopband + 1e-12)

    pass_max_db = float(passband_db.max())
    pass_min_db = float(passband_db.min())
    stop_max_db = float(stopband_db.max())

    return {
        "pass_max_db": pass_max_db,
        "pass_min_db": pass_min_db,
        "pass_ripple_db": pass_max_db - pass_min_db,
        "stop_max_db": stop_max_db,
        "stop_atten_db": -stop_max_db,
    }


def design_lpfs(filters: tuple, fs_hz: float) -> list[dict[str, object]]:
    """Design all FIR low-pass filters in a profile's LPF table."""
    results: list[dict[str, object]] = []
    nyquist = 0.5 * fs_hz

    for spec in filters:
        name = str(spec["name"])
        f_pass_hz = float(spec["f_pass_khz"]) * 1000.0
        f_stop_hz = float(spec["f_stop_khz"]) * 1000.0
        order = int(spec["order"])
        weight_pass = float(spec["weight_pass"])
        weight_stop = float(spec["weight_stop"])
        taps = lpf_num_taps(order)

        if not (0.0 < f_pass_hz < f_stop_hz < nyquist):
            raise ValueError(
                f"LPF {name}: require 0 < f_pass < f_stop < Fs/2; got "
                f"f_pass={f_pass_hz:g} Hz, f_stop={f_stop_hz:g} Hz, "
                f"Fs/2={nyquist:g} Hz"
            )
        if order < 1:
            raise ValueError(f"LPF {name}: order must be at least 1")
        if weight_pass <= 0.0 or weight_stop <= 0.0:
            raise ValueError(f"LPF {name}: weights must be positive")

        coefficients = design_fir_lpf(
            fs_hz=fs_hz,
            taps=taps,
            f_pass_hz=f_pass_hz,
            f_stop_hz=f_stop_hz,
            weight_pass=weight_pass,
            weight_stop=weight_stop,
        )
        metrics = measure_fir_lpf(
            coefficients, fs_hz, f_pass_hz, f_stop_hz
        )
        results.append(
            {
                "name": name,
                "f_pass_hz": f_pass_hz,
                "f_stop_hz": f_stop_hz,
                "order": order,
                "taps": taps,
                "weight_pass": weight_pass,
                "weight_stop": weight_stop,
                "coefficients": coefficients,
                "metrics": metrics,
            }
        )

    return results


def design_fir(
    fs_hz: float,
    taps: int,
    f_pass_hz: float,
    f_stop_hz: float,
    weight_pass: float,
    weight_stop: float,
    slope: float,
    gain: float,
) -> np.ndarray:
    """Design the FIR differentiator used by the FM demodulator.

    Uses scipy.signal.remez with type='differentiator':

        remez(taps, [0, f_pass, f_stop, Fs/2], [slope, 0],
              weight=[weight_pass, weight_stop], type='differentiator', fs=Fs)

    ``gain`` scales the coefficients after design.
    """
    bands = np.array([0.0, f_pass_hz, f_stop_hz, 0.5 * fs_hz], dtype=float)
    desired = np.array([slope, 0.0], dtype=float)

    coefficients = signal.remez(
        taps,
        bands,
        desired,
        weight=[weight_pass, weight_stop],
        type="differentiator",
        fs=fs_hz,
    )
    return np.asarray(coefficients, dtype=float) * float(gain)


def iir_gain_at(
    b: np.ndarray, a: np.ndarray, frequency_hz: float, fs_hz: float
) -> float:
    """Return the magnitude response of a first-order IIR filter at a frequency."""
    _, response = signal.freqz(b, a, worN=[frequency_hz], fs=fs_hz)
    return float(np.abs(response[0]))


def design_iir(
    fs_hz: float,
    f_corner_hz: float,
    f_high_hz: float,
    f_ref_hz: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Design first-order pre-emphasis and de-emphasis IIR filters.

        wc = 2*pi*f_corner
        w_high = 2*pi*f_high
        b, a = bilinear([1, wc], [1, w_high], Fs)
        b = b / a[0]
        a = a / a[0]
        gain_ref = |H(f_ref)|
        b = b / gain_ref
        b_de = a
        a_de = b
        b_de = b_de / a_de[0]
        a_de = a_de / a_de[0]

    Normalizing the pre-emphasis filter to 0 dB at f_ref makes the inverse
    de-emphasis filter also 0 dB at f_ref, and keeps the cascade exactly flat.
    The de-emphasis coefficients are renormalized so that a0 = 1, as required
    by the provided FirstOrderIIR structure.
    """
    wc = 2.0 * np.pi * f_corner_hz
    w_high = 2.0 * np.pi * f_high_hz

    b_pre, a_pre = signal.bilinear([1.0, wc], [1.0, w_high], fs_hz)

    # The bilinear transform normally returns a[0] = 1; normalize explicitly
    # so the generated coefficients always satisfy the FirstOrderIIR contract.
    b_pre = b_pre / a_pre[0]
    a_pre = a_pre / a_pre[0]

    # Normalize pre-emphasis to 0 dB at the reference frequency.
    gain_ref = iir_gain_at(b_pre, a_pre, f_ref_hz, fs_hz)
    if gain_ref == 0.0:
        raise ValueError(
            f"pre-emphasis gain at f_ref={f_ref_hz:g} Hz is zero; "
            "choose a different reference frequency"
        )
    b_pre = b_pre / gain_ref

    # De-emphasis is the inverse of the normalized pre-emphasis: swap
    # numerator/denominator and renormalize to a0 = 1 for FirstOrderIIR.
    b_de = a_pre.copy()
    a_de = b_pre.copy()

    b_de = b_de / a_de[0]
    a_de = a_de / a_de[0]

    return b_pre, a_pre, b_de, a_de


def validate_params(args: argparse.Namespace) -> None:
    """Validate command-line parameters before designing filters."""
    fs_hz = float(args.fs)
    nyquist = 0.5 * fs_hz

    if fs_hz <= 0.0:
        raise ValueError("--fs must be positive")

    if not (0.0 < args.f_pass < args.f_stop < nyquist):
        raise ValueError(
            "require 0 < --f-pass < --f-stop < Fs/2; got "
            f"f_pass={args.f_pass:g} Hz, f_stop={args.f_stop:g} Hz, "
            f"Fs/2={nyquist:g} Hz"
        )

    if args.fir_taps < 2:
        raise ValueError("--fir-taps must be at least 2")

    if args.fir_weight_pass <= 0.0 or args.fir_weight_stop <= 0.0:
        raise ValueError("--fir-weight-pass and --fir-weight-stop must be positive")

    if args.fir_gain <= 0.0:
        raise ValueError("--fir-gain must be positive")

    iir_nyquist = 0.5 * float(args.iir_fs)
    if not (0.0 < args.f_corner < iir_nyquist):
        raise ValueError(
            f"require 0 < --f-corner < IIR Fs/2; got f_corner={args.f_corner:g} Hz, "
            f"IIR Fs/2={iir_nyquist:g} Hz"
        )

    if args.iir_f_high <= args.f_corner:
        raise ValueError("--iir-f-high must be greater than --f-corner")

    if args.iir_f_high >= iir_nyquist:
        print(
            "WARNING: --iir-f-high is at or above IIR Nyquist; the digital pole "
            "will be very close to Nyquist.",
            file=sys.stderr,
        )

    if not (0.0 < args.f_ref < iir_nyquist):
        raise ValueError(
            f"require 0 < --f-ref < IIR Fs/2; got f_ref={args.f_ref:g} Hz, "
            f"IIR Fs/2={iir_nyquist:g} Hz"
        )


def iir_dc_gain(b: np.ndarray, a: np.ndarray) -> float:
    """Return the DC gain of a first-order IIR filter."""
    return float((b[0] + b[1]) / (a[0] + a[1]))


def report(
    args: argparse.Namespace,
    prof: dict,
    fir: np.ndarray,
    b_pre: np.ndarray,
    a_pre: np.ndarray,
    b_de: np.ndarray,
    a_de: np.ndarray,
    lpf_filters: list[dict[str, object]],
) -> None:
    """Print a short summary of the designed filters."""
    brief = prof["brief"]
    print(f"{brief} filter parameters:")
    print(f"  Fs              : {args.fs:g} Hz")
    if float(args.iir_fs) != float(args.fs):
        print(f"  IIR Fs          : {args.iir_fs:g} Hz")
    print(f"  FIR taps        : {len(fir)}")
    print(f"  FIR passband    : 0 .. {args.f_pass:g} Hz")
    print(f"  FIR stopband    : {args.f_stop:g} .. {0.5 * args.fs:g} Hz")
    print(
        "  FIR weights     : "
        f"pass={args.fir_weight_pass:g}, stop={args.fir_weight_stop:g}"
    )
    print(f"  FIR gain        : {args.fir_gain:g}")
    print(f"  IIR corner      : {args.f_corner:g} Hz")
    print(f"  IIR high pole   : {args.iir_f_high:g} Hz")
    print(f"  IIR ref freq    : {args.f_ref:g} Hz")
    print(
        "  Pre-emphasis    : "
        f"b0={b_pre[0]:.9g}, b1={b_pre[1]:.9g}, a1={a_pre[1]:.9g}"
    )
    print(
        "  De-emphasis     : "
        f"b0={b_de[0]:.9g}, b1={b_de[1]:.9g}, a1={a_de[1]:.9g}"
    )
    gain_pre_ref = iir_gain_at(b_pre, a_pre, args.f_ref, args.iir_fs)
    gain_de_ref = iir_gain_at(b_de, a_de, args.f_ref, args.iir_fs)
    print(
        "  IIR gain @ ref  : "
        f"pre={gain_pre_ref:.6g} ({20.0 * np.log10(gain_pre_ref + 1e-12):.3f} dB), "
        f"de={gain_de_ref:.6g} ({20.0 * np.log10(gain_de_ref + 1e-12):.3f} dB)"
    )
    print(
        "  IIR DC gains    : "
        f"pre={iir_dc_gain(b_pre, a_pre):.6g}, "
        f"de={iir_dc_gain(b_de, a_de):.6g}"
    )

    if lpf_filters:
        print(f"{brief} FIR LPF filters:")
        for lpf in lpf_filters:
            metrics = lpf["metrics"]
            assert isinstance(metrics, dict)
            print(
                f"  [{lpf['name']}] order={lpf['order']}, taps={lpf['taps']}, "
                f"pass=0 .. {lpf['f_pass_hz']:g} Hz, "
                f"stop={lpf['f_stop_hz']:g} .. {0.5 * args.fs:g} Hz, "
                f"weights=pass {lpf['weight_pass']:g} / stop {lpf['weight_stop']:g}"
            )
            print(
                "    passband        : "
                f"{metrics['pass_min_db']:.3f} .. {metrics['pass_max_db']:.3f} dB "
                f"(ripple {metrics['pass_ripple_db']:.3f} dB p-p)"
            )
            print(
                "    stopband        : "
                f"max {metrics['stop_max_db']:.3f} dB "
                f"(attenuation {metrics['stop_atten_db']:.3f} dB)"
            )


def build_header(
    args: argparse.Namespace,
    prof: dict,
    fir: np.ndarray,
    b_pre: np.ndarray,
    a_pre: np.ndarray,
    b_de: np.ndarray,
    a_de: np.ndarray,
    lpf_filters: list[dict[str, object]],
) -> str:
    """Build the C header contents."""
    symbol = prof["symbol"]
    base = prof["base"]
    brief = prof["brief"]

    lpf_summary: list[str] = []
    for lpf in lpf_filters:
        metrics = lpf["metrics"]
        assert isinstance(metrics, dict)
        lpf_summary.extend(
            [
                f" * LPF {lpf['name']:<4}: order={lpf['order']}, "
                f"taps={lpf['taps']}, pass=0 .. {lpf['f_pass_hz']:g} Hz,",
                f" *           stop={lpf['f_stop_hz']:g} .. {0.5 * args.fs:g} Hz, "
                f"weights=pass {lpf['weight_pass']:g} / "
                f"stop {lpf['weight_stop']:g},",
                f" *           passband ripple {metrics['pass_ripple_db']:.3f} dB p-p, "
                f"stopband attenuation {metrics['stop_atten_db']:.3f} dB",
            ]
        )

    lpf_lines: list[str] = []
    for lpf in lpf_filters:
        name = str(lpf["name"])
        identifier = filter_c_identifier(name)
        macro_name = f"{symbol}_LPF_{identifier.upper()}_NUM_TAPS"
        coefficients = lpf["coefficients"]
        assert isinstance(coefficients, np.ndarray)
        metrics = lpf["metrics"]
        assert isinstance(metrics, dict)
        lpf_lines.extend(
            [
                "/*",
                f" * {brief} FIR LPF {name}: order={lpf['order']}, "
                f"taps={lpf['taps']}.",
                f" * Passband 0 .. {lpf['f_pass_hz']:g} Hz; "
                f"stopband {lpf['f_stop_hz']:g} .. {0.5 * args.fs:g} Hz.",
                " * Passband response is 0 dB (no post-design normalization).",
                f" * Passband ripple: {metrics['pass_ripple_db']:.3f} dB p-p "
                f"({metrics['pass_min_db']:.3f} .. "
                f"{metrics['pass_max_db']:.3f} dB).",
                f" * Stopband attenuation: {metrics['stop_atten_db']:.3f} dB "
                f"(max {metrics['stop_max_db']:.3f} dB).",
                " * Coefficients are in natural order.",
                " */",
                f"#define {macro_name} {lpf['taps']}u",
                "",
                f"static const float {base}_lpf_{identifier}_coeffs"
                f"[{macro_name}] = {{",
                format_array(coefficients),
                "};",
                "",
            ]
        )

    lpf_header_lines: list[str] = []
    if lpf_filters:
        lpf_header_lines.append(
            f" * {brief} FIR LPF filters (0 dB passband, no normalization):"
        )
        lpf_header_lines.extend(lpf_summary)
        lpf_header_lines.append(" *")

    iir_fs_line = []
    if float(args.iir_fs) != float(args.fs):
        iir_fs_line.append(f" * IIR Fs          : {args.iir_fs:g} Hz")

    lines = [
        "#pragma once",
        "",
        "/*",
        f" * {prof['headline']}",
        " * Generated by tools/gen_fm_filters.py. Do not edit by hand.",
        " *",
        f" * Fs              : {args.fs:g} Hz",
        *iir_fs_line,
        f" * FIR taps        : {len(fir)}",
        f" * FIR passband    : 0 .. {args.f_pass:g} Hz",
        f" * FIR stopband    : {args.f_stop:g} .. {0.5 * args.fs:g} Hz",
        " * FIR weights     : "
        f"pass={args.fir_weight_pass:g}, stop={args.fir_weight_stop:g}",
        f" * FIR gain        : {args.fir_gain:g}",
        f" * IIR corner      : {args.f_corner:g} Hz",
        f" * IIR high pole   : {args.iir_f_high:g} Hz",
        f" * IIR ref freq    : {args.f_ref:g} Hz (0 dB normalization)",
        " *",
        *lpf_header_lines,
        " * FIR differentiator:",
        " *   Designed for the phase signal produced by atan2(). For an",
        " *   unwrapped phase in radians, the raw FIR output approximates",
        " *   f / Fs (cycles per sample). Multiply the output by Fs to obtain",
        " *   Hz, or set --fir-gain Fs to fold that scaling into the",
        " *   coefficients.",
        " *",
        " * IIR pre/de-emphasis:",
        " *   The FirstOrderIIR type is expected to be defined by the includer:",
        " *",
        " *     typedef struct {",
        " *         float b0, b1;",
        " *         float a1;",
        " *         float x1, y1;",
        " *     } FirstOrderIIR;",
        " *",
        " *   The provided first_order_iir() difference equation assumes a0 = 1:",
        " *",
        " *     y = b0*x + b1*x1 - a1*y1",
        " *",
        " *   Usage:",
        " *",
        f" *     static FirstOrderIIR pre = {symbol}_PRE_IIR_INIT;",
        f" *     static FirstOrderIIR de  = {symbol}_DE_IIR_INIT;",
        " *",
        " *   Both filters are normalized to 0 dB at f_ref. De-emphasis",
        " *   coefficients are renormalized so that a0 = 1. The",
        " *   pre/de-emphasis cascade is flat (unity gain).",
        " */",
        "",
        "/*",
        " * FIR coefficients are in natural order. Reverse them only if the",
        " * target FIR implementation requires time-reversed coefficients.",
        " */",
        f"#define {symbol}_FIR_NUM_TAPS {len(fir)}u",
        "",
        f"static const float {base}_fir_coeffs[{symbol}_FIR_NUM_TAPS] = {{",
        format_array(fir),
        "};",
        "",
        *lpf_lines,
        "/*",
        " * First-order IIR initializers. States x1 and y1 start at zero.",
        " */",
        f"#define {symbol}_PRE_IIR_INIT {{ .b0 = {format_float(b_pre[0])}, "
        f".b1 = {format_float(b_pre[1])}, .a1 = {format_float(a_pre[1])}, "
        ".x1 = 0.0f, .y1 = 0.0f }",
        f"#define {symbol}_DE_IIR_INIT  {{ .b0 = {format_float(b_de[0])}, "
        f".b1 = {format_float(b_de[1])}, .a1 = {format_float(a_de[1])}, "
        ".x1 = 0.0f, .y1 = 0.0f }",
        "",
    ]
    return "\n".join(lines)


def plot_response(
    args: argparse.Namespace,
    prof: dict,
    fir: np.ndarray,
    b_pre: np.ndarray,
    a_pre: np.ndarray,
    b_de: np.ndarray,
    a_de: np.ndarray,
    plot_path: Path,
) -> None:
    """Generate and save frequency-response plots."""
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError as exc:
        raise RuntimeError(
            "--plot requires matplotlib; install it or omit --plot"
        ) from exc

    fig, (ax_fir, ax_iir) = plt.subplots(2, 1, figsize=(10.0, 8.0))

    w_fir, h_fir = signal.freqz(fir, worN=8192, fs=args.fs)
    ideal_response = w_fir / (args.fs)
    ax_fir.plot(w_fir, 20.0 * np.log10(np.abs(h_fir) + 1e-12))
    ax_fir.plot(w_fir, 20.0 * np.log10(ideal_response + 1e-12))
    ax_fir.axvline(
        args.f_pass, color="g", linestyle="--", label=f"f_pass={args.f_pass:g} Hz"
    )
    ax_fir.axvline(
        args.f_stop, color="r", linestyle="--", label=f"f_stop={args.f_stop:g} Hz"
    )
    ax_fir.set_title(prof["plot_title_fir"])
    ax_fir.set_xlabel("Frequency (Hz)")
    ax_fir.set_ylabel("Magnitude (dB)")
    ax_fir.set_xlim(0.0, 0.5 * args.fs)
    ax_fir.set_ylim(-80, 0)
    ax_fir.grid(True)
    ax_fir.legend()

    w_iir, h_pre = signal.freqz(b_pre, a_pre, worN=4096, fs=args.iir_fs)
    _, h_de = signal.freqz(b_de, a_de, worN=4096, fs=args.iir_fs)
    positive = w_iir > 0.0
    ax_iir.semilogx(
        w_iir[positive],
        20.0 * np.log10(np.abs(h_pre[positive]) + 1e-12),
        label="Pre-emphasis",
    )
    ax_iir.semilogx(
        w_iir[positive],
        20.0 * np.log10(np.abs(h_de[positive]) + 1e-12),
        label="De-emphasis",
    )
    ax_iir.semilogx(
        w_iir[positive],
        20.0 * np.log10(np.abs(h_pre[positive] * h_de[positive]) + 1e-12),
        "--",
        label="Cascade",
    )
    ax_iir.axvline(
        args.f_corner, color="r", linestyle=":", label=f"f_corner={args.f_corner:g} Hz"
    )
    ax_iir.axvline(
        args.f_ref, color="b", linestyle="-.", label=f"f_ref={args.f_ref:g} Hz"
    )
    ax_iir.set_title(prof["plot_title_iir"])
    ax_iir.set_xlabel("Frequency (Hz)")
    ax_iir.set_ylabel("Magnitude (dB)")
    ax_iir.grid(True, which="both")
    ax_iir.legend()

    fig.tight_layout()
    plot_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(plot_path, dpi=150)
    plt.close(fig)


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Generate FM FIR/IIR filter coefficients as a C header. "
            "Select the target with --profile."
        )
    )
    parser.add_argument(
        "--profile",
        choices=sorted(PROFILES),
        default="nbfm",
        help="filter profile to generate (default: nbfm)",
    )
    parser.add_argument(
        "--fs",
        type=float,
        default=None,
        help="FIR (differentiator/LPF) sample rate in Hz (default: profile)",
    )
    parser.add_argument(
        "--iir-fs",
        type=float,
        default=None,
        help="IIR sample rate in Hz; defaults to --fs",
    )
    parser.add_argument(
        "--f-corner",
        type=float,
        default=None,
        help="IIR pre/de-emphasis corner frequency in Hz (default: profile)",
    )
    parser.add_argument(
        "--iir-f-high",
        type=float,
        default=None,
        help="IIR analog high-frequency pole in Hz (default: profile)",
    )
    parser.add_argument(
        "--f-ref",
        type=float,
        default=None,
        help="IIR reference frequency for 0 dB normalization (default: profile)",
    )
    parser.add_argument(
        "--f-pass",
        type=float,
        default=None,
        help="FIR differentiator passband edge in Hz (default: profile)",
    )
    parser.add_argument(
        "--f-stop",
        type=float,
        default=None,
        help="FIR differentiator stopband edge in Hz (default: profile)",
    )
    parser.add_argument(
        "--fir-taps",
        type=int,
        default=None,
        help="number of FIR differentiator coefficients (default: profile)",
    )
    parser.add_argument(
        "--fir-weight-pass",
        type=float,
        default=None,
        help="FIR Remez passband weight (default: profile)",
    )
    parser.add_argument(
        "--fir-weight-stop",
        type=float,
        default=None,
        help="FIR Remez stopband weight (default: profile)",
    )
    parser.add_argument(
        "--fir-slope",
        type=float,
        default=None,
        help="FIR Remez desired passband slope for type='differentiator' "
        "(default: profile)",
    )
    parser.add_argument(
        "--fir-gain",
        type=float,
        default=None,
        help="post-design FIR coefficient gain (default: profile)",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=None,
        help="C header output path (default: profile)",
    )
    parser.add_argument(
        "--plot",
        action="store_true",
        help="generate frequency-response plots (requires matplotlib)",
    )
    parser.add_argument(
        "--plot-file",
        type=Path,
        default=None,
        help="plot output path (default: profile)",
    )
    return parser.parse_args(argv)


def apply_profile_defaults(args: argparse.Namespace, prof: dict) -> None:
    """Fill any unset CLI option from the selected profile."""
    defaults = prof["defaults"]
    for key in ("fs", "f_corner", "iir_f_high", "f_ref", "f_pass", "f_stop",
                "fir_taps", "fir_weight_pass", "fir_weight_stop", "fir_slope",
                "fir_gain"):
        if getattr(args, key) is None:
            setattr(args, key, defaults[key])
    if args.iir_fs is None:
        args.iir_fs = args.fs
    if args.output is None:
        args.output = prof["output"]
    if args.plot_file is None:
        args.plot_file = prof["plot"]


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    prof = PROFILES[args.profile]
    apply_profile_defaults(args, prof)

    try:
        validate_params(args)
        fir = design_fir(
            fs_hz=args.fs,
            taps=args.fir_taps,
            f_pass_hz=args.f_pass,
            f_stop_hz=args.f_stop,
            weight_pass=args.fir_weight_pass,
            weight_stop=args.fir_weight_stop,
            slope=args.fir_slope,
            gain=args.fir_gain,
        )
        b_pre, a_pre, b_de, a_de = design_iir(
            fs_hz=args.iir_fs,
            f_corner_hz=args.f_corner,
            f_high_hz=args.iir_f_high,
            f_ref_hz=args.f_ref,
        )
        lpf_filters = design_lpfs(prof["lpf_filters"], args.fs)
    except ValueError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1

    report(args, prof, fir, b_pre, a_pre, b_de, a_de, lpf_filters)

    header = build_header(args, prof, fir, b_pre, a_pre, b_de, a_de, lpf_filters)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(header, encoding="utf-8")
    print(f"Wrote {args.output}")

    if args.plot:
        try:
            plot_response(args, prof, fir, b_pre, a_pre, b_de, a_de, args.plot_file)
        except RuntimeError as exc:
            print(f"ERROR: {exc}", file=sys.stderr)
            return 1
        print(f"Wrote plot {args.plot_file}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
