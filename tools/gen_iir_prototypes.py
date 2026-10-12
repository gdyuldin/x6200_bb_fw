#!/usr/bin/env python3
"""
Generate analog IIR prototypes for the App/SDR IIR module.

Two families share one generator and one descriptor table:

  * cheby2  -- Chebyshev Type II, equiripple stopband, flat passband.
  * ellip   -- elliptic (Cauer), equiripple passband and stopband, the
               narrowest transition band for a given order.

The configuration lives in the FAMILIES table below.  Two headers are written:

  * App/SDR/sdr_iir_orders.h
        The SdrIirStages enum: one constant per supported section count, its
        value being the section count itself (analog order / 2).  This is the
        public key the runtime and its callers select a filter by, shared by
        every family because stages = order / 2 holds for all of them.

  * App/SDR/sdr_iir_prototypes.h
        The per-family analog sections plus a single descriptor table keyed by
        (family, section count), so the runtime needs no hand-written lists.

Each prototype is normalized to a passband-side reference point at 1 rad/s: for
cheby2 the chosen attenuation level (REF_DB, a point in the long transition
band), for the elliptic family the passband ripple edge.  Each second-order
section is emitted as a C initializer in the order

    {b0, b1, b2, a0, a1, a2}

for

    H(s) = (b0 + b1*s + b2*s^2) / (a0 + a1*s + a2*s^2).

The overall gain is folded into the last section, so the product of all
sections is the complete analog prototype.  Each descriptor also carries the
normalized stopband-edge frequency (where the prototype reaches the family's
stopband attenuation) relative to that reference point, so runtime can scale by
the stopband edge instead of the reference point.

The script also emits per-filter noise measures: the normalized LPF equivalent
noise bandwidth (ENBW), the normalized HPF blocking bandwidth, and the LPF
attenuation at its ENBW.  These describe the response rather than a band edge;
runtime uses them as a starting guess and as a common level reference, and
measures the real digital band from the coefficients.

    python3 tools/gen_iir_prototypes.py

The script resolves its paths from its own location, so it can be run from any
working directory.  It prints a short validation summary to stderr.
"""

from __future__ import annotations

import sys
from pathlib import Path
from typing import Iterable, TextIO

import numpy as np
import scipy.signal as sig
from scipy.integrate import quad
from scipy.optimize import brentq

# Chebyshev II: equiripple stopband at ATTEN_DB, normalized so the REF_DB point
# sits at 1 rad/s.  REF_DB is a compromise inside the long transition band -- a
# defined, monotone reference below the stopband edge.
#
# Elliptic: RIPPLE_DB passband ripple and STOP_DB stopband attenuation, both
# equiripple; normalized to the passband ripple edge at 1 rad/s.  The
# transition is the narrowest possible for the order, at the cost of group-delay
# peaking near the edge.
FAMILIES = (
    {
        "name": "cheby2",
        "symbol": "cheby2",
        "orders": (4, 6, 8, 10, 12),
        "atten_db": 100.0,
        "ref_db": -9.0,
        "ref_text": "100 dB stopband, -9 dB at 1 rad/s",
    },
    {
        "name": "ellip",
        "symbol": "ellip",
        "orders": (8, 10, 12),
        "ripple_db": 0.02,
        "stop_db": 80.0,
        "ref_text": "0.02 dB ripple, 80 dB stopband, passband edge at 1 rad/s",
    },
)

TYPES = ("lpf", "hpf")

REPO_ROOT = Path(__file__).resolve().parent.parent
SDR_DIR = REPO_ROOT / "App" / "SDR"
ORDERS_PATH = SDR_DIR / "sdr_iir_orders.h"
OUTPUT_PATH = SDR_DIR / "sdr_iir_prototypes.h"

# The names of the emitted per-order prototype arrays.
ARRAY_NAME = {
    (family["name"], order, kind): f"sdr_{family['symbol']}_{kind}{order}"
    for family in FAMILIES for order in family["orders"] for kind in TYPES
}

# Identifier of the public enum constant for each stage count.
STAGES_CONST = {
    order: f"SDR_IIR_STAGES_{order // 2}"
    for family in FAMILIES for order in family["orders"]
}

# Every stage count across all families, ascending, for the shared orders enum.
ALL_STAGES = sorted({order // 2 for family in FAMILIES
                     for order in family["orders"]})

# The stage count backing the SdrIirStages enum type name.
STAGES_TYPE = "SdrIirStages"

# Type name of the generated descriptor and the family selector it carries.
PROTO_TYPE = "SdrIirProto"
FAMILY_TYPE = "SdrIirFamily"

# The generated descriptor table.
PROTO_TABLE = "sdr_iir_protos"


def _family_enum(family: dict) -> str:
    return "SDR_IIR_FAMILY_" + family["name"].upper()


def _format_float(value: float) -> str:
    """Format a float as a C float literal."""
    if float(value) == 0.0:
        return "0.0f"
    text = f"{float(value):.9g}"
    if "." not in text and "e" not in text and "E" not in text:
        text += ".0"
    return text + "f"


def _lowpass_magnitude(w: float, z: np.ndarray, p: np.ndarray, k: float) -> float:
    s = 1j * w
    return float(abs(k * np.prod(s - z) / np.prod(s - p)))


def _hpf_magnitude(w: float, z: np.ndarray, p: np.ndarray, k: float) -> float:
    return _lowpass_magnitude(w, z, p, k)


def _enbw_lpf(z: np.ndarray, p: np.ndarray, k: float) -> float:
    """Normalized one-sided ENBW of the LPF prototype, rad/s.

    Integral of (|H(jw)|/|H(0)|)^2 over the analog frequency axis.  The
    digital ENBW is smaller near Nyquist because bilinear warping compresses
    this tail, so runtime measures the real band instead of scaling this
    constant.
    """
    h0 = _lowpass_magnitude(0.0, z, p, k)
    integrand = lambda w: (_lowpass_magnitude(w, z, p, k) / h0) ** 2
    value, _ = quad(integrand, 0.0, np.inf, limit=400)
    return float(value)


def _block_hpf(z: np.ndarray, p: np.ndarray, k: float) -> float:
    """Normalized equivalent blocking bandwidth of the HPF prototype, rad/s.

    Integral of 1 - (|H(jw)|/|H(inf)|)^2.  The direct passband ENBW of a
    high-pass diverges, while this removed-noise measure converges and is the
    quantity useful for a noise budget.
    """
    h_inf = abs(k)
    integrand = lambda w: 1.0 - (_hpf_magnitude(w, z, p, k) / h_inf) ** 2
    value, _ = quad(integrand, 0.0, np.inf, limit=400)
    return float(value)


def _att_at(w: float, z: np.ndarray, p: np.ndarray, k: float) -> float:
    """Magnitude response at w relative to DC, in dB (positive = attenuated)."""
    h0 = _lowpass_magnitude(0.0, z, p, k)
    return float(-20.0 * np.log10(_lowpass_magnitude(w, z, p, k) / h0))


def _find_th(z: np.ndarray, p: np.ndarray, k: float, ref_db: float) -> float:
    """Return the ref_db frequency of the raw Chebyshev II prototype."""
    target = 10.0 ** (ref_db / 20.0)

    ws = np.linspace(1e-6, 1.0, 20000)
    mags = np.array([_lowpass_magnitude(w, z, p, k) for w in ws])
    idx = int(np.argmax(mags < target))
    return float(brentq(lambda w: _lowpass_magnitude(w, z, p, k) - target,
                        float(ws[idx - 1]), float(ws[idx])))


def _find_stop(z: np.ndarray, p: np.ndarray, k: float, stop_db: float) -> float:
    """Return the first frequency above the passband where att reaches stop_db."""
    h0 = _lowpass_magnitude(0.0, z, p, k)
    target = h0 * 10.0 ** (-stop_db / 20.0)

    ws = np.linspace(1.0, 4.0, 400000)
    mags = np.array([_lowpass_magnitude(w, z, p, k) for w in ws])
    idx = int(np.argmax(mags <= target))
    return float(brentq(lambda w: _lowpass_magnitude(w, z, p, k) - target,
                        float(ws[idx - 1]), float(ws[idx])))


def _normalized_lpf(family: dict, order: int) -> tuple[np.ndarray, np.ndarray, float, float]:
    """Return the normalized LPF prototype zpk and its normalized stop edge.

    The prototype is scaled so its passband-side reference point is 1 rad/s; the
    returned stop edge is the (normalized) frequency where the family's stopband
    attenuation is first reached, always > 1.
    """
    if family["name"] == "cheby2":
        z, p, k = sig.cheb2ap(order, family["atten_db"])
        # The raw prototype reaches atten_db at 1 rad/s; move its ref_db point
        # to 1 rad/s, so the stopband edge moves to 1/w3.
        w3 = _find_th(z, p, k, family["ref_db"])
        z = z / w3
        p = p / w3
        return z, p, k, 1.0 / w3

    z, p, k = sig.ellipap(order, family["ripple_db"], family["stop_db"])
    # ellipap is already normalized to the passband ripple edge at 1 rad/s.
    stop_edge = _find_stop(z, p, k, family["stop_db"])
    return z, p, k, stop_edge


def _make_sections(family: dict, order: int, kind: str) -> tuple[np.ndarray, float, float]:
    """Return analog sections, the overall gain and the stopband-edge frequency."""
    z, p, k, stop_edge_lpf = _normalized_lpf(family, order)

    if kind == "hpf":
        z, p, k = sig.lp2hp_zpk(z, p, k, wo=1.0)
        # lp2hp maps w -> 1/w, so the low-pass stop edge maps to 1/stop_edge.
        stop_edge = 1.0 / stop_edge_lpf
    else:
        stop_edge = stop_edge_lpf

    # Positive-imaginary representatives, paired in ascending order.  Even
    # orders have no real poles/zeros, so every section is a conjugate pair.
    zp = sorted(z[np.imag(z) > 0.0], key=lambda value: abs(float(np.imag(value))))
    pp = sorted(p[np.imag(p) > 0.0], key=lambda value: abs(float(np.imag(value))))

    if len(zp) != order // 2 or len(pp) != order // 2:
        raise RuntimeError(f"unexpected pole/zero count for order {order}")

    sections = []
    for zero, pole in zip(zp, pp):
        # (s - z)(s - z*) = s^2 - 2*Re(z)*s + |z|^2
        b0 = float(abs(zero) ** 2)
        b1 = float(-2.0 * np.real(zero))
        b2 = 1.0
        a0 = float(abs(pole) ** 2)
        a1 = float(-2.0 * np.real(pole))
        a2 = 1.0
        sections.append([b0, b1, b2, a0, a1, a2])

    # Fold the overall gain into the last section.
    for index in range(3):
        sections[-1][index] *= k

    return np.array(sections, dtype=float), float(k), float(stop_edge)


def _emit_array(out: TextIO, name: str, sections: np.ndarray, order: int,
                kind: str, family: dict) -> None:
    stages = order // 2
    print(f"/* {name}: order {order} {kind.upper()}, {family['ref_text']} */",
          file=out)
    print(f"static const float {name}[{stages}][6] = {{", file=out)
    for section in sections:
        values = ", ".join(_format_float(value) for value in section)
        print(f"    {{{values}}},", file=out)
    print("};", file=out)
    print(file=out)


def _emit_data_header(out: TextIO) -> None:
    print("/* Auto-generated by tools/gen_iir_prototypes.py. Do not edit. */", file=out)
    print("#pragma once", file=out)
    print(file=out)
    print("#include <stdint.h>", file=out)
    print(file=out)
    print("/*", file=out)
    print(" * Analog IIR prototypes, one section array per family/order/type.", file=out)
    print(" * Each section is {b0, b1, b2, a0, a1, a2} in descending powers of s:", file=out)
    print(" *   H(s) = (b0 + b1*s + b2*s^2) / (a0 + a1*s + a2*s^2).", file=out)
    print(" * Prototypes are normalized so their passband-side reference point", file=out)
    print(" * is 1 rad/s; its meaning is per family (see gen_iir_prototypes.py).", file=out)
    print(" * The overall gain is folded into the last section.  Sections are", file=out)
    print(" * ordered for cascade use.", file=out)
    print(" *", file=out)
    print(" * The descriptor table below is the only runtime entry point: one", file=out)
    print(" * entry per filter, keyed by family and section count.  The stop-edge", file=out)
    print(" * and ENBW values are separate measures of the response relative to", file=out)
    print(" * the reference point, not band edges; the high-pass has no finite", file=out)
    print(" * analog ENBW, so it carries a blocking bandwidth instead.", file=out)
    print(" */", file=out)
    print(file=out)


def _emit_orders_header(out: TextIO) -> None:
    print("/* Auto-generated by tools/gen_iir_prototypes.py. Do not edit. */", file=out)
    print("#pragma once", file=out)
    print(file=out)
    print("/*", file=out)
    print(" * Biquad section count of each supported IIR filter, shared by every", file=out)
    print(" * family (stages = analog order / 2).  One constant per generated", file=out)
    print(" * prototype; the value is the section count itself, so an SdrIirStages", file=out)
    print(" * value is usable directly as an array bound and as a CMSIS numStages", file=out)
    print(" * argument.", file=out)
    print(" */", file=out)
    print("typedef enum {", file=out)
    for stages in ALL_STAGES:
        print(f"    SDR_IIR_STAGES_{stages} = {stages},", file=out)
    print(f"}} {STAGES_TYPE};", file=out)
    print(file=out)


def _emit_descriptor_table(out: TextIO, rows: list) -> None:
    print("/* Filter family, disambiguates a section count across prototypes. */", file=out)
    print(f"typedef enum {{", file=out)
    for family in FAMILIES:
        print(f"    {_family_enum(family)} = {FAMILIES.index(family)},", file=out)
    print(f"}} {FAMILY_TYPE};", file=out)
    print(file=out)
    print("/*", file=out)
    print(" * Runtime descriptor per filter, keyed by (family, section count).  The", file=out)
    print(" * runtime searches this table, so those two fields are the only key it", file=out)
    print(" * needs and the per-order arrays above stay independent of each other.", file=out)
    print(" */", file=out)
    print("typedef struct {", file=out)
    print(f"    {FAMILY_TYPE} family;", file=out)
    print("    uint8_t      stages;         /* biquad sections, analog order / 2 */", file=out)
    print("    const float *lpf;            /* [stages][6] analog LPF sections */", file=out)
    print("    const float *hpf;            /* [stages][6] analog HPF sections */", file=out)
    print("    float        stop_edge_lpf;  /* LPF stopband edge relative to the ref point */", file=out)
    print("    float        stop_edge_hpf;  /* HPF stopband edge relative to the ref point */", file=out)
    print("    float        noise_norm_lpf; /* LPF passband ENBW, analog, rad/s */", file=out)
    print("    float        noise_norm_hpf; /* HPF blocked bandwidth, analog, rad/s */", file=out)
    print("    float        level_att_db;   /* attenuation at the LPF ENBW, positive dB */", file=out)
    print(f"}} {PROTO_TYPE};", file=out)
    print(file=out)
    print(f"static const {PROTO_TYPE} {PROTO_TABLE}[{len(rows)}] = {{", file=out)
    for row in rows:
        print("    {{{family}, {stages}u, &{lpf}[0][0], &{hpf}[0][0], {sl}, {sh}, "
              "{nl}, {nh}, {att}}},  /* {sc} */".format(
                  family=_family_enum(row["family"]),
                  stages=row["stages"],
                  lpf=row["lpf"],
                  hpf=row["hpf"],
                  sl=_format_float(row["stop_edge_lpf"]),
                  sh=_format_float(row["stop_edge_hpf"]),
                  nl=_format_float(row["noise_norm_lpf"]),
                  nh=_format_float(row["noise_norm_hpf"]),
                  att=_format_float(row["level_att_db"]),
                  sc=STAGES_CONST[row["order"]]),
              file=out)
    print("};", file=out)
    print(file=out)


def _emit_summary() -> None:
    """Print a short validation summary to stderr."""
    for family in FAMILIES:
        print(f"{family['name']}: {family['ref_text']}", file=sys.stderr)
        for order in family["orders"]:
            for kind in TYPES:
                z, p, k, stop_edge_lpf = _normalized_lpf(family, order)
                if kind == "hpf":
                    z, p, k = sig.lp2hp_zpk(z, p, k, wo=1.0)
                    stop_edge = 1.0 / stop_edge_lpf
                else:
                    stop_edge = stop_edge_lpf
                print(f"  stages={order // 2:2d} order={order:2d} "
                      f"type={kind.upper()} gain={k:.9g} "
                      f"stop_edge={stop_edge:.9g}", file=sys.stderr)
            z_lp, p_lp, k_lp, _ = _normalized_lpf(family, order)
            z_hp, p_hp, k_hp = sig.lp2hp_zpk(z_lp, p_lp, k_lp, wo=1.0)
            enbw = _enbw_lpf(z_lp, p_lp, k_lp)
            block = _block_hpf(z_hp, p_hp, k_hp)
            att = _att_at(enbw, z_lp, p_lp, k_lp)
            print(f"  stages={order // 2:2d} enbw_norm={enbw:.9g} "
                  f"att@enbw={att:.6g}dB block_norm={block:.9g} "
                  f"1/enbw={1.0 / enbw:.9g}", file=sys.stderr)


def main(argv: Iterable[str]) -> int:
    _ = list(argv)
    rows = []
    with OUTPUT_PATH.open("w") as out:
        _emit_data_header(out)
        for family in FAMILIES:
            for order in family["orders"]:
                stop_edge = {}
                for kind in TYPES:
                    sections, _, stop_edge[kind] = _make_sections(family, order, kind)
                    _emit_array(out, ARRAY_NAME[(family["name"], order, kind)],
                                sections, order, kind, family)

                z_lp, p_lp, k_lp, _ = _normalized_lpf(family, order)
                z_hp, p_hp, k_hp = sig.lp2hp_zpk(z_lp, p_lp, k_lp, wo=1.0)
                enbw = _enbw_lpf(z_lp, p_lp, k_lp)
                rows.append({
                    "family": family,
                    "order": order,
                    "stages": order // 2,
                    "lpf": ARRAY_NAME[(family["name"], order, "lpf")],
                    "hpf": ARRAY_NAME[(family["name"], order, "hpf")],
                    "stop_edge_lpf": stop_edge["lpf"],
                    "stop_edge_hpf": stop_edge["hpf"],
                    "noise_norm_lpf": enbw,
                    "noise_norm_hpf": _block_hpf(z_hp, p_hp, k_hp),
                    "level_att_db": _att_at(enbw, z_lp, p_lp, k_lp),
                })

        _emit_descriptor_table(out, rows)

    with ORDERS_PATH.open("w") as out:
        _emit_orders_header(out)

    _emit_summary()
    print(f"wrote {ORDERS_PATH}", file=sys.stderr)
    print(f"wrote {OUTPUT_PATH}", file=sys.stderr)
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
