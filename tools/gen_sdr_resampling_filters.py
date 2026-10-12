#!/usr/bin/env python3
"""
Generate coefficients for the SDR resampling chain.

This script is intentionally configured by editing the FILTERS list below.
It writes one C header with one coefficient array per configured stage:

  - half-band FIR decimators (Sdr_HalfbandDecim_*), decimation by 2,
  - an elliptic IIR decimator (CMSIS biquad cascade),
  - FIR interpolators for arm_fir_interpolate_f32.

All sample rates and band edges are in kHz. Every decimator and interpolator
changes the rate by 2.

Run:
    python3 tools/gen_sdr_resampling_filters.py

Output:
    App/SDR/sdr_resampling_filters.h
"""

from pathlib import Path

import numpy as np
import scipy.signal as sig

# ---------------------------------------------------------------------------
# Configuration.
#
# "kind" is one of:
#   "halfband_decim": half-band FIR decimator using Sdr_HalfbandDecim_*.
#   "fir_interp":     linear-phase FIR interpolator using arm_fir_interpolate_f32.
#   "iir_ellip":      elliptic IIR decimator using arm_biquad_cascade_df1_f32.
#
# "order" is the requested filter order N; the number of taps is N+1. Half-band
# stages round the tap count UP to an odd number (a half-band filter must have
# an odd tap count). Interpolators round UP to a multiple of 4.
#
# Half-band stages may only set fs_in, fs_out, f_pass and f_stop. Their band
# edges must satisfy f_pass + f_stop == fs_in / 2; the equal-weight lowpass is
# then forced into a half-band structure (see design_halfband).
# ---------------------------------------------------------------------------
FILTERS = [
    {
        "kind": "iir_ellip",
        "name": "sdr_decim_384_90",
        "fs_in_khz": 384,
        "fs_out_khz": 192,
        "f_pass_khz": 90,
        "f_stop_khz": 102,
        "order": 8,
        "ripple_db": 0.4,
        "stop_db": 70,
        "note": "WFM output",
    },
    {
        "kind": "halfband_decim",
        "name": "sdr_decim_384_9",
        "fs_in_khz": 384,
        "fs_out_khz": 192,
        "f_pass_khz": 9,
        "f_stop_khz": 183,
        "order": 6,
        "note": "intermediate light stage",
    },
    {
        "kind": "halfband_decim",
        "name": "sdr_decim_192_9",
        "fs_in_khz": 192,
        "fs_out_khz": 96,
        "f_pass_khz": 9,
        "f_stop_khz": 87,
        "order": 10,
        "note": "intermediate stage",
    },
    {
        "kind": "halfband_decim",
        "name": "sdr_decim_96_9",
        "fs_in_khz": 96,
        "fs_out_khz": 48,
        "f_pass_khz": 9,
        "f_stop_khz": 39,
        "order": 14,
        "note": "intermediate stage",
    },
    {
        "kind": "halfband_decim",
        "name": "sdr_decim_48_9",
        "fs_in_khz": 48,
        "fs_out_khz": 24,
        "f_pass_khz": 9,
        "f_stop_khz": 15,
        "order": 42,
        "note": "AM/NBFM downsample / audio out upsample",
    },
    {
        "kind": "halfband_decim",
        "name": "sdr_decim_24_5",
        "fs_in_khz": 24,
        "fs_out_khz": 12,
        "f_pass_khz": 5,
        "f_stop_khz": 7,
        "order": 62,
        "note": "SSB/eSSB/CW output / audio upsample",
    },
    {
        "kind": "halfband_decim",
        "name": "sdr_decim_192_15",
        "fs_in_khz": 192,
        "fs_out_khz": 96,
        "f_pass_khz": 15,
        "f_stop_khz": 81,
        "order": 14,
        "note": "WFM audio 192 -> 96 kHz",
    },
    {
        "kind": "halfband_decim",
        "name": "sdr_decim_96_15",
        "fs_in_khz": 96,
        "fs_out_khz": 48,
        "f_pass_khz": 15,
        "f_stop_khz": 33,
        "order": 26,
        "note": "WFM audio 96 -> 48 kHz",
    },
    {
        "kind": "fir_interp",
        "name": "sdr_interp_24_5",
        "fs_in_khz": 24,
        "fs_out_khz": 12,
        "f_pass_khz": 5,
        "f_stop_khz": 7,
        "order": 39,
        "weight_pass": 1.0,
        "weight_stop": 144.0,
        "note": "Output audio and IQ interpolation (level-preserving)",
    },
    {
        "kind": "fir_interp",
        "name": "sdr_interp_48_9",
        "fs_in_khz": 48,
        "fs_out_khz": 24,
        "f_pass_khz": 9,
        "f_stop_khz": 15,
        "order": 34,
        "weight_pass": 1.0,
        "weight_stop": 5.76,
        "note": "Output audio and IQ interpolation (level-preserving)",
    },
    {
        "kind": "iir_ellip",
        "name": "sdr_spectrum_zoom",
        "fs_in_khz": 100,
        "fs_out_khz": 50,
        "f_pass_khz": 24,
        "f_stop_khz": 26,
        "order": 10,
        "ripple_db": 0.5,
        "stop_db": 80,
        "note": "spectrum zoom decimator, reused per halving stage",
    },
]

# Fixed output path, relative to the project root (parent of tools/).
OUTPUT_PATH = Path(__file__).resolve().parent.parent / "App/SDR/sdr_resampling_filters.h"


def format_float(x: float) -> str:
    """Format a float as a C float literal with enough precision."""
    s = f"{x:.9g}"
    if "." not in s and "e" not in s and "E" not in s:
        s += ".0"
    return s + "f"


def round_taps_to_multiple_of_4(order: int) -> tuple[int, int]:
    """Return (num_taps, adjusted_order) with num_taps rounded up to 4."""
    num_taps = order + 1
    num_taps = ((num_taps + 3) // 4) * 4
    return num_taps, num_taps - 1


def round_taps_to_odd(order: int) -> tuple[int, int]:
    """Return (num_taps, adjusted_order) with num_taps rounded up to odd."""
    num_taps = order + 1
    if num_taps % 2 == 0:
        num_taps += 1
    return num_taps, num_taps - 1


def measure_response(w: np.ndarray, h: np.ndarray, f_pass: float,
                     f_stop: float) -> dict:
    """Measure passband ripple and stopband attenuation on a response grid."""
    pass_mask = w <= f_pass
    stop_mask = w >= f_stop
    pass_mag = np.abs(h[pass_mask])
    stop_mag = np.abs(h[stop_mask])

    pass_max_db = 20.0 * np.log10(np.max(pass_mag)) if pass_mag.size else float("nan")
    pass_min_db = 20.0 * np.log10(np.min(pass_mag)) if pass_mag.size else float("nan")
    stop_atten_db = -20.0 * np.log10(np.max(stop_mag)) if stop_mag.size else float("nan")

    return {
        "pass_max_db": pass_max_db,
        "pass_min_db": pass_min_db,
        "pass_ripple_db": pass_max_db - pass_min_db,
        "stop_atten_db": stop_atten_db,
    }


def design_halfband(cfg: dict, decim: int) -> dict:
    """Design a half-band FIR decimator with forced half-band structure."""
    name = cfg["name"]
    fs_in = cfg["fs_in_khz"]
    f_pass = cfg["f_pass_khz"]
    f_stop = cfg["f_stop_khz"]
    order = cfg["order"]

    # A half-band lowpass has its transition band centred on fs_in / 4, i.e.
    # f_pass + f_stop == fs_in / 2. Anything else is not a half-band design.
    if abs((f_pass + f_stop) - fs_in / 2.0) > 1e-9:
        raise ValueError(
            f"{name}: half-band requires f_pass + f_stop == fs_in/2 "
            f"(got {f_pass} + {f_stop} = {f_pass + f_stop}, "
            f"fs_in/2 = {fs_in / 2.0})"
        )

    num_taps, adjusted_order = round_taps_to_odd(order)
    center = (num_taps - 1) // 2

    bands = np.array([0.0, f_pass / fs_in, f_stop / fs_in, 0.5])
    desired = np.array([1.0, 0.0])
    b = sig.remez(num_taps, bands, desired, weight=[1.0, 1.0], fs=1.0)

    # Force the exact half-band structure. remez does not place the zeros when
    # the transition band is wide, so record how far it was from doing so.
    zero_idx = [center + 2 * m for m in range(1, center // 2 + 1)]
    zero_idx += [center - 2 * m for m in range(1, center // 2 + 1)]
    collapsed = max((abs(float(b[i])) for i in zero_idx), default=0.0)

    b = b.copy()
    b[center] = 0.5
    for i in zero_idx:
        b[i] = 0.0

    # Normalize DC gain to 1.0 using the off-centre taps only.
    off_center = np.ones(num_taps, dtype=bool)
    off_center[center] = False
    b[off_center] *= 0.5 / np.sum(b[off_center])

    # Non-zero off-centre taps share one parity; validate the assumption.
    nonzero = [i for i in range(num_taps) if i != center and b[i] != 0.0]
    phase = nonzero[0] % 2
    if any(i % 2 != phase for i in nonzero):
        raise ValueError(f"{name}: half-band taps are not on a single parity")

    # CMSIS-DSP expects time-reversed coefficients. Reversal preserves parity.
    coeffs = b[::-1]

    w, h = sig.freqz(b, 1.0, worN=16384, fs=1.0)
    measured = measure_response(w * fs_in, h, f_pass, f_stop)

    return {
        "kind": "halfband_decim",
        "cfg": cfg,
        "decim": decim,
        "num_taps": num_taps,
        "adjusted_order": adjusted_order,
        "center": center,
        "phase": phase,
        "collapsed": collapsed,
        "coeffs": coeffs,
        **measured,
    }


def design_fir_interp(cfg: dict, decim: int) -> dict:
    """Design a time-reversed FIR interpolator for arm_fir_interpolate_f32."""
    name = cfg["name"]
    fs_in = cfg["fs_in_khz"]
    f_pass = cfg["f_pass_khz"]
    f_stop = cfg["f_stop_khz"]
    order = cfg["order"]
    w_pass = cfg["weight_pass"]
    w_stop = cfg["weight_stop"]

    if w_pass <= 0.0 or w_stop <= 0.0:
        raise ValueError(f"{name}: weights must be > 0")

    num_taps, adjusted_order = round_taps_to_multiple_of_4(order)

    bands = np.array([0.0, f_pass / fs_in, f_stop / fs_in, 0.5])
    desired = np.array([1.0, 0.0])
    weights = np.array([w_pass, w_stop])

    b = sig.remez(num_taps, bands, desired, weight=weights, fs=1.0)
    coeffs = b[::-1]

    w, h = sig.freqz(b, 1.0, worN=16384, fs=1.0)
    measured = measure_response(w * fs_in, h, f_pass, f_stop)

    return {
        "kind": "fir_interp",
        "cfg": cfg,
        "decim": decim,
        "num_taps": num_taps,
        "adjusted_order": adjusted_order,
        "coeffs": coeffs,
        **measured,
    }


def design_iir(cfg: dict, decim: int) -> dict:
    """Design an elliptic IIR decimator as a CMSIS DF1 biquad cascade."""
    name = cfg["name"]
    fs_in = cfg["fs_in_khz"]
    f_pass = cfg["f_pass_khz"]
    f_stop = cfg["f_stop_khz"]
    order = cfg["order"]
    ripple_db = cfg["ripple_db"]
    stop_db = cfg["stop_db"]

    if ripple_db <= 0.0 or stop_db <= 0.0:
        raise ValueError(f"{name}: ripple and stopband must be > 0 dB")

    sos = sig.ellip(order, ripple_db, stop_db, f_pass / (fs_in / 2.0),
                    btype="low", output="sos")
    num_sections = sos.shape[0]

    # CMSIS DF1 uses {b0, b1, b2, a1, a2} with the denominator sign negated.
    cmsis = np.empty((num_sections, 5))
    cmsis[:, 0] = sos[:, 0]
    cmsis[:, 1] = sos[:, 1]
    cmsis[:, 2] = sos[:, 2]
    cmsis[:, 3] = -sos[:, 4]
    cmsis[:, 4] = -sos[:, 5]

    w, h = sig.sosfreqz(sos, worN=16384, fs=fs_in)
    measured = measure_response(w, h, f_pass, f_stop)

    return {
        "kind": "iir_ellip",
        "cfg": cfg,
        "decim": decim,
        "order": order,
        "num_sections": num_sections,
        "sos": cmsis.reshape(-1),
        **measured,
    }


def design_filter(cfg: dict) -> dict:
    """Validate one stage and return its design results."""
    name = cfg["name"]
    fs_in = cfg["fs_in_khz"]
    fs_out = cfg["fs_out_khz"]
    f_pass = cfg["f_pass_khz"]
    f_stop = cfg["f_stop_khz"]
    order = cfg["order"]
    kind = cfg.get("kind", "halfband_decim")

    if fs_in <= 0 or fs_out <= 0:
        raise ValueError(f"{name}: sample rates must be positive")
    if fs_in % fs_out != 0:
        raise ValueError(f"{name}: fs_in must be an integer multiple of fs_out")
    decim = fs_in // fs_out
    if decim != 2:
        raise ValueError(f"{name}: this script expects decimation factor 2, got {decim}")

    if not (0.0 < f_pass < f_stop < fs_in / 2.0):
        raise ValueError(
            f"{name}: require 0 < f_pass < f_stop < fs_in/2 "
            f"(got f_pass={f_pass}, f_stop={f_stop}, fs_in/2={fs_in/2.0})"
        )

    # To avoid aliasing into the passband after decimation by M:
    #   f_stop <= fs_in/M - f_pass = fs_out - f_pass
    max_stop = fs_out - f_pass
    if f_stop > max_stop + 1e-12:
        raise ValueError(
            f"{name}: f_stop={f_stop} kHz exceeds {max_stop} kHz; "
            f"aliasing would fall into the passband after decimation by {decim}"
        )

    if order < 1:
        raise ValueError(f"{name}: order must be >= 1")

    if kind == "halfband_decim":
        return design_halfband(cfg, decim)
    if kind == "fir_interp":
        return design_fir_interp(cfg, decim)
    if kind == "iir_ellip":
        return design_iir(cfg, decim)
    raise ValueError(f"{name}: unknown kind {kind!r}")


def format_array(name: str, coeffs: np.ndarray) -> str:
    """Format coefficients as a C initializer list."""
    lines = [f"    {format_float(c)}," for c in coeffs]
    return "\n".join(lines)


def build_halfband(res: dict) -> list[str]:
    """Build the header text for one half-band decimator."""
    cfg = res["cfg"]
    name = cfg["name"]
    macro = name.upper()
    num_taps = res["num_taps"]

    return [
        "/*",
        f" * {name}",
        " *   Kind            : half-band FIR, decimation by 2",
        f" *   Input rate      : {cfg['fs_in_khz']} kHz",
        f" *   Output rate     : {cfg['fs_out_khz']} kHz (decimation by {res['decim']})",
        f" *   Passband edge   : {cfg['f_pass_khz']} kHz",
        f" *   Stopband edge   : {cfg['f_stop_khz']} kHz",
        f" *   Requested order : N={cfg['order']}",
        f" *   Adjusted order  : N={res['adjusted_order']} ({num_taps} taps, odd)",
        f" *   Structure       : forced half-band zeros "
        f"(max |w| before snap {res['collapsed']:.6g})",
        f" *   Measured        : passband ripple {res['pass_ripple_db']:.3f} dB, "
        f"stopband attenuation {res['stop_atten_db']:.3f} dB",
        f" *   Purpose         : {cfg['note']}",
        " */",
        f"#define {macro}_NUM_TAPS {num_taps}u",
        f"#define {macro}_HALFBAND_CENTER {res['center']}u",
        f"#define {macro}_HALFBAND_PHASE {res['phase']}u",
        f'_Static_assert(({macro}_NUM_TAPS & 1u) == 1u, "{name} tap count must be odd");',
        "",
        f"static const float32_t {name}_coeffs[{macro}_NUM_TAPS] = {{",
        format_array(name, res["coeffs"]),
        "};",
        "",
    ]


def build_interp(res: dict) -> list[str]:
    """Build the header text for one FIR interpolator."""
    cfg = res["cfg"]
    name = cfg["name"]
    macro = name.upper()
    num_taps = res["num_taps"]

    return [
        "/*",
        f" * {name}",
        " *   Kind            : FIR interpolator, interpolation by 2",
        f" *   Input rate      : {cfg['fs_out_khz']} kHz",
        f" *   Output rate     : {cfg['fs_in_khz']} kHz (interpolation by {res['decim']})",
        f" *   Passband edge   : {cfg['f_pass_khz']} kHz",
        f" *   Stopband edge   : {cfg['f_stop_khz']} kHz",
        f" *   Taps            : {num_taps} (multiple of L=2)",
        f" *   Weights         : passband={cfg['weight_pass']:g}, "
        f"stopband={cfg['weight_stop']:g}",
        f" *   Measured        : passband ripple {res['pass_ripple_db']:.3f} dB, "
        f"stopband attenuation {res['stop_atten_db']:.3f} dB",
        f" *   Coefficients    : scale-invariant design * 2.0",
        f" *   Purpose         : {cfg['note']}",
        " */",
        f"#define {macro}_NUM_TAPS {num_taps}u",
        "",
        f"static const float32_t {name}_coeffs[{macro}_NUM_TAPS] = {{",
        format_array(name, res["coeffs"] * 2.0),
        "};",
        "",
    ]


def build_header(results: list[dict]) -> str:
    """Build the complete C header text."""
    parts = [
        "#pragma once",
        "",
        "#include <arm_math.h>",
        "",
        "/*",
        " * SDR resampling coefficients.",
        " * Generated by tools/gen_sdr_resampling_filters.py. Do not edit by hand.",
        " *",
        " * Half-band FIR decimators use Sdr_HalfbandDecim_* (time-reversed",
        " * coefficients, one zeroed parity). Elliptic IIR stages are CMSIS biquad",
        " * cascades (arm_biquad_cascade_df1_f32) with {b0, b1, b2, a1, a2} sections.",
        " * Interpolators use arm_fir_interpolate_f32. Every stage changes the rate",
        " * by 2. Band edges in kHz.",
        " */",
        "",
    ]

    for res in results:
        kind = res["kind"]
        if kind == "iir_ellip":
            cfg = res["cfg"]
            name = cfg["name"]
            sections = res["num_sections"]
            macro = name.upper() + "_IIR_SECTIONS"

            parts.extend([
                "/*",
                f" * {name}",
                " *   Kind            : elliptic IIR, CMSIS biquad cascade DF1",
                f" *   Input rate      : {cfg['fs_in_khz']} kHz",
                f" *   Output rate     : {cfg['fs_out_khz']} kHz (decimation by {res['decim']})",
                f" *   Passband edge   : {cfg['f_pass_khz']} kHz",
                f" *   Stopband edge   : {cfg['f_stop_khz']} kHz",
                f" *   Order           : N={res['order']} ({sections} sections)",
                f" *   Design          : passband ripple {cfg['ripple_db']:g} dB, "
                f"stopband attenuation {cfg['stop_db']:g} dB",
                f" *   Measured        : passband ripple {res['pass_ripple_db']:.3f} dB, "
                f"stopband attenuation {res['stop_atten_db']:.3f} dB",
                f" *   Purpose         : {cfg['note']}",
                " *   Coefficients    : CMSIS DF1 {b0, b1, b2, a1, a2} per section",
                " */",
                f"#define {macro} {sections}u",
                "",
                f"static const float32_t {name}_iir_sos[{macro} * 5u] = {{",
                format_array(name, res["sos"]),
                "};",
                "",
            ])
        elif kind == "halfband_decim":
            parts.extend(build_halfband(res))
        elif kind == "fir_interp":
            parts.extend(build_interp(res))
        else:
            raise ValueError(f"unknown kind {kind!r}")

    return "\n".join(parts)


def main() -> int:
    results = []
    for cfg in FILTERS:
        try:
            res = design_filter(cfg)
        except ValueError as exc:
            print(f"ERROR: {exc}")
            return 1
        results.append(res)

        print(f"{cfg['name']}: {cfg['fs_in_khz']} -> {cfg['fs_out_khz']} kHz, "
              f"M={res['decim']}, kind={res['kind']}")
        print(f"  passband : 0 .. {cfg['f_pass_khz']} kHz")
        print(f"  stopband : {cfg['f_stop_khz']} .. {cfg['fs_in_khz'] / 2.0:g} kHz")
        if res["kind"] == "iir_ellip":
            print(f"  order N  : {res['order']} ({res['num_sections']} sections)")
            print(f"  design   : ripple {cfg['ripple_db']:g} dB, "
                  f"stopband {cfg['stop_db']:g} dB")
        else:
            print(f"  order N  : {cfg['order']} -> {res['adjusted_order']} "
                  f"({res['num_taps']} taps)")
        if res["kind"] == "halfband_decim":
            print(f"  halfband  : center={res['center']}, phase={res['phase']}, "
                  f"collapsed={res['collapsed']:.6g}")
        print(f"  ripple   : {res['pass_ripple_db']:.3f} dB")
        print(f"  atten    : {res['stop_atten_db']:.3f} dB")
        print()

    header = build_header(results)
    OUTPUT_PATH.parent.mkdir(parents=True, exist_ok=True)
    OUTPUT_PATH.write_text(header, encoding="utf-8")
    print(f"Wrote {OUTPUT_PATH}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
