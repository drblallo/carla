#!/usr/bin/env python3
"""
Realtime CSV line visualizer with pygame.

- Accepts a CSV file path via argparse
- Reloads the file every second
- Uses only the last 20 rows
- Draws one line per column, left-to-right across time
- Vertically stacks columns evenly on the screen
- Scales values using the global min/max observed since startup
- Prints the latest value under each line

CSV format example:
1,2,3
2,3,4
3,4,5
"""

from __future__ import annotations

import argparse
import csv
import sys
import time
from pathlib import Path
from typing import List, Optional

import pygame


WINDOW_WIDTH = 1200
WINDOW_HEIGHT = 800
FPS = 60
RELOAD_INTERVAL_SECONDS = 1.0
MAX_ROWS = 20

BACKGROUND = (18, 18, 18)
GRID = (50, 50, 50)
TEXT = (230, 230, 230)
AXIS = (120, 120, 120)

LINE_COLORS = [
    (255, 99, 132),
    (54, 162, 235),
    (255, 206, 86),
    (75, 192, 192),
    (153, 102, 255),
    (255, 159, 64),
    (199, 199, 199),
    (83, 102, 255),
]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Render CSV columns as scrolling line plots with pygame."
    )
    parser.add_argument(
        "file",
        type=Path,
        help="Path to the CSV file",
    )
    parser.add_argument(
        "--width",
        type=int,
        default=WINDOW_WIDTH,
        help=f"Window width (default: {WINDOW_WIDTH})",
    )
    parser.add_argument(
        "--height",
        type=int,
        default=WINDOW_HEIGHT,
        help=f"Window height (default: {WINDOW_HEIGHT})",
    )
    return parser.parse_args()


def load_csv_last_rows(path: Path, max_rows: int = MAX_ROWS) -> List[List[float]]:
    if not path.exists():
        raise FileNotFoundError(f"File not found: {path}")

    rows: List[List[float]] = []

    with path.open("r", newline="") as f:
        reader = csv.reader(f)
        for line_number, row in enumerate(reader, start=1):
            if not row:
                continue

            try:
                values = [float(cell.strip()) for cell in row]
            except ValueError as exc:
                raise ValueError(
                    f"Invalid numeric value on line {line_number}: {row}"
                ) from exc

            rows.append(values)

    if not rows:
        return []

    expected_len = len(rows[0])
    for i, row in enumerate(rows, start=1):
        if len(row) != expected_len:
            raise ValueError(
                f"Inconsistent column count on line {i}: "
                f"expected {expected_len}, got {len(row)}"
            )

    return rows[-max_rows:]


def update_observed_range(
    rows: List[List[float]],
    observed_min: Optional[float],
    observed_max: Optional[float],
) -> tuple[Optional[float], Optional[float]]:
    if not rows:
        return observed_min, observed_max

    current_min = min(min(row) for row in rows)
    current_max = max(max(row) for row in rows)

    if observed_min is None or current_min < observed_min:
        observed_min = current_min
    if observed_max is None or current_max > observed_max:
        observed_max = current_max

    return observed_min, observed_max


def value_to_y(
    value: float,
    global_min: float,
    global_max: float,
    region_top: float,
    region_height: float,
) -> int:
    padding = 18
    usable_height = max(10.0, region_height - 2 * padding)

    if global_max == global_min:
        return int(region_top + region_height / 2)

    # Higher values should be higher on screen.
    normalized = (value - global_min) / (global_max - global_min)
    y = region_top + padding + (1.0 - normalized) * usable_height
    return int(y)


def draw_centered_text(
    surface: pygame.Surface,
    font: pygame.font.Font,
    text: str,
    x: int,
    y: int,
    color: tuple[int, int, int] = TEXT,
) -> None:
    rendered = font.render(text, True, color)
    rect = rendered.get_rect(center=(x, y))
    surface.blit(rendered, rect)


def draw_plot(
    surface: pygame.Surface,
    rows: List[List[float]],
    observed_min: Optional[float],
    observed_max: Optional[float],
    font: pygame.font.Font,
    small_font: pygame.font.Font,
) -> None:
    surface.fill(BACKGROUND)

    width, height = surface.get_size()
    left_margin = 60
    right_margin = 30
    top_margin = 30
    bottom_margin = 50

    if not rows:
        draw_centered_text(
            surface,
            font,
            "No valid data yet",
            width // 2,
            height // 2,
        )
        return

    num_cols = len(rows[0])
    if num_cols == 0:
        draw_centered_text(
            surface,
            font,
            "CSV has zero columns",
            width // 2,
            height // 2,
        )
        return

    assert observed_min is not None
    assert observed_max is not None

    plot_width = width - left_margin - right_margin
    plot_height = height - top_margin - bottom_margin
    region_height = plot_height / num_cols

    # Grid lines separating each series region.
    for col in range(num_cols + 1):
        y = int(top_margin + col * region_height)
        pygame.draw.line(surface, GRID, (left_margin, y), (width - right_margin, y), 1)

    # X spacing across visible rows.
    num_points = len(rows)
    if num_points == 1:
        x_positions = [left_margin + plot_width // 2]
    else:
        x_positions = [
            int(left_margin + i * (plot_width / (num_points - 1)))
            for i in range(num_points)
        ]

    for col in range(num_cols):
        region_top = top_margin + col * region_height
        region_center_y = int(region_top + region_height / 2)
        color = LINE_COLORS[col % len(LINE_COLORS)]

        # Draw center guide for the band.
        pygame.draw.line(
            surface,
            AXIS,
            (left_margin, region_center_y),
            (width - right_margin, region_center_y),
            1,
        )

        points = []
        for row_idx, row in enumerate(rows):
            value = row[col]
            x = x_positions[row_idx]
            y = value_to_y(
                value,
                observed_min,
                observed_max,
                region_top,
                region_height,
            )
            points.append((x, y))

        if len(points) >= 2:
            pygame.draw.lines(surface, color, False, points, 2)
        elif len(points) == 1:
            pygame.draw.circle(surface, color, points[0], 3)

        latest_value = rows[-1][col]
        latest_x, latest_y = points[-1]

        # Latest point marker.
        pygame.draw.circle(surface, color, (latest_x, latest_y), 5)

        # Column label at left.
        label = f"x{col + 1}"
        label_surface = small_font.render(label, True, TEXT)
        label_rect = label_surface.get_rect(midright=(left_margin - 10, region_center_y))
        surface.blit(label_surface, label_rect)

        # Latest numeric value under the latest point.
        value_text = f"{latest_value:g}"
        value_surface = small_font.render(value_text, True, TEXT)
        value_rect = value_surface.get_rect(midtop=(latest_x, latest_y + 8))
        surface.blit(value_surface, value_rect)

    range_text = f"Observed min: {observed_min:g}    max: {observed_max:g}    rows shown: {len(rows)}"
    surface.blit(small_font.render(range_text, True, TEXT), (10, height - 28))


def main() -> int:
    args = parse_args()

    pygame.init()
    pygame.display.set_caption("CSV Realtime Plot")
    screen = pygame.display.set_mode((args.width, args.height), pygame.RESIZABLE)
    clock = pygame.time.Clock()
    font = pygame.font.SysFont(None, 32)
    small_font = pygame.font.SysFont(None, 22)

    observed_min: Optional[float] = None
    observed_max: Optional[float] = None
    rows: List[List[float]] = []
    last_reload = 0.0
    last_error: Optional[str] = None

    running = True
    while running:
        now = time.monotonic()

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        if now - last_reload >= RELOAD_INTERVAL_SECONDS:
            last_reload = now
            try:
                new_rows = load_csv_last_rows(args.file, MAX_ROWS)
                rows = new_rows
                observed_min, observed_max = update_observed_range(
                    rows,
                    observed_min,
                    observed_max,
                )
                last_error = None
            except Exception as exc:
                last_error = str(exc)

        draw_plot(screen, rows, observed_min, observed_max, font, small_font)

        if last_error:
            error_surface = small_font.render(f"Error: {last_error}", True, (255, 120, 120))
            screen.blit(error_surface, (10, 10))

        pygame.display.flip()
        clock.tick(FPS)

    pygame.quit()
    return 0


if __name__ == "__main__":
    sys.exit(main())
