#!/usr/bin/env python3
"""Draw the measured HW3 arena and landmark ground truth.

The figure is a top-down plan, not a plot of the SLAM estimate.  Positions are
read from truth_map.yaml.  The tag bars are drawn with the physical 0.166 m
side length; truth_map.yaml stores 0.156 m only as the detector's effective
PnP side length.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import yaml
from matplotlib.patches import Circle, Rectangle


ROOT = Path(__file__).resolve().parent
PHYSICAL_TAG_SIZE_M = 0.166
ARENA_SIZE_M = 3.0
FEET_PER_M = 3.280839895
INK = "#202020"
WALL = "#303030"
GRID = "#d8d8d8"
NORMAL = "#245a8d"
TABLE_ALT = "#f1f3f5"


def load_landmarks(path: Path) -> tuple[float, list[dict]]:
    data = yaml.safe_load(path.read_text())
    arena = float(data.get("arena_size", ARENA_SIZE_M))
    landmarks = sorted(data["landmarks"], key=lambda item: int(item["id"]))
    return arena, landmarks


def normal_from_yaw(yaw_deg: float) -> np.ndarray:
    yaw = np.deg2rad(float(yaw_deg))
    return np.array([np.cos(yaw), np.sin(yaw)], dtype=float)


def draw_tag_bar(ax, centre: np.ndarray, normal: np.ndarray, size: float) -> None:
    """Draw the top view of a vertical tag as a bar on the arena wall."""
    tangent = np.array([-normal[1], normal[0]])
    half = 0.5 * size
    p0 = centre - half * tangent
    p1 = centre + half * tangent
    ax.plot(
        [p0[0], p1[0]],
        [p0[1], p1[1]],
        color="black",
        linewidth=5.0,
        solid_capstyle="butt",
        clip_on=False,
        zorder=5,
    )


def draw_tag_id(ax, centre: np.ndarray, normal: np.ndarray, tag_id: int) -> None:
    label_at = centre + 0.42 * normal
    ax.plot(
        [centre[0] + 0.06 * normal[0], label_at[0] - 0.085 * normal[0]],
        [centre[1] + 0.06 * normal[1], label_at[1] - 0.085 * normal[1]],
        color=NORMAL,
        linewidth=1.2,
        solid_capstyle="butt",
        zorder=6,
    )
    arrow_end = centre + 0.31 * normal
    ax.annotate(
        "",
        xy=arrow_end,
        xytext=centre + 0.11 * normal,
        arrowprops=dict(arrowstyle="-|>", color=NORMAL, lw=1.2, mutation_scale=8),
        zorder=7,
    )
    ax.add_patch(
        Circle(
            label_at,
            radius=0.085,
            facecolor="white",
            edgecolor=INK,
            linewidth=0.9,
            zorder=8,
        )
    )
    ax.text(
        label_at[0],
        label_at[1] - 0.005,
        str(tag_id),
        ha="center",
        va="center",
        fontsize=8.2,
        color=INK,
        zorder=9,
    )


def draw_dimension(
    ax,
    start: tuple[float, float],
    end: tuple[float, float],
    text: str,
    text_offset: tuple[float, float],
    rotation: float = 0.0,
) -> None:
    ax.annotate(
        "",
        xy=end,
        xytext=start,
        arrowprops=dict(arrowstyle="<|-|>", color=INK, lw=0.9, mutation_scale=7),
        annotation_clip=False,
        zorder=4,
    )
    ax.text(
        (start[0] + end[0]) / 2 + text_offset[0],
        (start[1] + end[1]) / 2 + text_offset[1],
        text,
        ha="center",
        va="center",
        rotation=rotation,
        fontsize=8.2,
        color=INK,
        clip_on=False,
    )


def draw_arena(ax, arena: float, landmarks: list[dict]) -> None:
    ax.set_facecolor("white")
    ax.add_patch(
        Rectangle(
            (0.0, 0.0),
            arena,
            arena,
            facecolor="#fcfcfc",
            edgecolor=WALL,
            linewidth=2.2,
            zorder=1,
        )
    )

    ax.set_xticks(np.arange(0.0, arena + 1e-9, 1.0))
    ax.set_yticks(np.arange(0.0, arena + 1e-9, 1.0))
    ax.set_xticks(np.arange(0.5, arena, 0.5), minor=True)
    ax.set_yticks(np.arange(0.5, arena, 0.5), minor=True)
    ax.grid(which="minor", color=GRID, linewidth=0.55, linestyle=(0, (2, 3)), zorder=0)
    ax.tick_params(axis="both", which="major", labelsize=8.4, length=3.5, width=0.8, pad=2.0)
    ax.tick_params(axis="both", which="minor", length=0)

    ax.spines["bottom"].set_position(("data", 0.0))
    ax.spines["left"].set_position(("data", 0.0))
    ax.spines["bottom"].set_linewidth(0.0)
    ax.spines["left"].set_linewidth(0.0)
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)

    # Units are given by the dimension lines and by the figure caption.  Avoid
    # floating axis labels so that the drawing stays compact in the report.
    ax.set_xlabel("")
    ax.set_ylabel("")

    draw_dimension(
        ax,
        (0.0, -0.27),
        (arena, -0.27),
        rf"{arena:.2f} m  ({arena * FEET_PER_M:.2f} ft)",
        (0.0, -0.115),
    )
    draw_dimension(
        ax,
        (-0.22, 0.0),
        (-0.22, arena),
        rf"{arena:.2f} m  ({arena * FEET_PER_M:.2f} ft)",
        (-0.095, 0.0),
        rotation=90,
    )

    for item in landmarks:
        tag_id = int(item["id"])
        centre = np.array([float(item["x"]), float(item["y"])])
        normal = normal_from_yaw(float(item["yaw_deg"]))
        draw_tag_bar(ax, centre, normal, PHYSICAL_TAG_SIZE_M)
        draw_tag_id(ax, centre, normal, tag_id)

    ax.set_xlim(-0.42, arena + 0.30)
    ax.set_ylim(-0.46, arena + 0.28)
    ax.set_aspect("equal", adjustable="box")


def draw_table(ax, landmarks: list[dict]) -> None:
    ax.set_axis_off()
    ax.set_xlim(0.0, 1.0)
    ax.set_ylim(0.0, 1.0)

    ax.text(
        0.0,
        0.965,
        "Ground-truth tag centres",
        fontsize=9.2,
        fontweight="bold",
        color=INK,
        ha="left",
        va="top",
    )
    ax.text(
        0.0,
        0.905,
        "arena frame, metres",
        fontsize=7.8,
        color="#555555",
        ha="left",
        va="top",
    )

    cell_text = []
    for item in landmarks:
        yaw = float(item["yaw_deg"])
        yaw_text = rf"${yaw:.0f}^\circ$"
        cell_text.append(
            [
                str(int(item["id"])),
                f"{float(item['x']):.2f}",
                f"{float(item['y']):.2f}",
                yaw_text,
            ]
        )

    table = ax.table(
        cellText=cell_text,
        colLabels=["tag", r"$x$", r"$y$", r"yaw"],
        colWidths=[0.18, 0.25, 0.25, 0.32],
        cellLoc="center",
        colLoc="center",
        bbox=[0.0, 0.22, 1.0, 0.61],
    )
    table.auto_set_font_size(False)
    table.set_fontsize(8.1)

    for (row, _col), cell in table.get_celld().items():
        cell.set_linewidth(0.45)
        cell.set_edgecolor("#b8b8b8")
        cell.set_height(0.075)
        if row == 0:
            cell.set_facecolor("#e9ecef")
            cell.set_text_props(fontweight="bold", color=INK)
        elif row % 2 == 0:
            cell.set_facecolor(TABLE_ALT)
        else:
            cell.set_facecolor("white")

    ax.text(
        0.0,
        0.155,
        r"yaw is counter-clockwise from $+x$." + "\n" + "Arrows point into the arena.",
        fontsize=7.6,
        color="#444444",
        ha="left",
        va="top",
        linespacing=1.45,
    )
    ax.text(
        0.0,
        0.065,
        r"Tag side: 0.166 m.",
        fontsize=7.6,
        color="#444444",
        ha="left",
        va="top",
    )


def make_figure(truth_path: Path, out_dir: Path) -> tuple[Path, Path]:
    arena, landmarks = load_landmarks(truth_path)

    plt.rcParams.update(
        {
            "font.family": "serif",
            "font.serif": ["DejaVu Serif"],
            "mathtext.fontset": "cm",
            "axes.unicode_minus": True,
            "pdf.fonttype": 42,
            "ps.fonttype": 42,
        }
    )

    fig = plt.figure(figsize=(7.15, 5.15), facecolor="white")
    ax_map = fig.add_axes([0.115, 0.195, 0.485, 0.745])
    ax_table = fig.add_axes([0.620, 0.205, 0.355, 0.700])

    draw_arena(ax_map, arena, landmarks)
    draw_table(ax_table, landmarks)

    out_dir.mkdir(parents=True, exist_ok=True)
    png_path = out_dir / "hw3_truth_map.png"
    pdf_path = out_dir / "hw3_truth_map.pdf"
    fig.savefig(png_path, dpi=300, bbox_inches="tight", pad_inches=0.06)
    fig.savefig(pdf_path, bbox_inches="tight", pad_inches=0.06)
    plt.close(fig)
    return png_path, pdf_path


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--truth", type=Path, default=ROOT / "truth_map.yaml")
    parser.add_argument("--out-dir", type=Path, default=ROOT / "figures")
    args = parser.parse_args()

    png_path, pdf_path = make_figure(args.truth, args.out_dir)
    print(f"wrote {png_path}")
    print(f"wrote {pdf_path}")


if __name__ == "__main__":
    main()
