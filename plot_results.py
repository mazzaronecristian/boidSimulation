#!/usr/bin/env python3
"""Genera confronti tra benchmark sequenziali e paralleli.

File riconosciuti:
  benchmark_sequential_<esecuzioni>.csv
  benchmark_parallel_<scheduling>_<chunk_size>_<esecuzioni>.csv
"""

from __future__ import annotations

import argparse
import csv
import math
import re
import sys
from collections import defaultdict
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

try:
    import matplotlib.pyplot as plt
except ImportError as exc:
    raise SystemExit(
        "matplotlib non e' installato. Eseguire: "
        "python3 -m pip install matplotlib"
    ) from exc


SEQUENTIAL_RE = re.compile(r"^benchmark_sequential_(\d+)\.csv$", re.I)
PARALLEL_RE = re.compile(
    r"^benchmark_parallel_([^_]+)_([^_]+)_(\d+)\.csv$", re.I
)

ChunkSize = int | str


@dataclass(frozen=True)
class Summary:
    mean: float
    stddev: float
    samples: int


def positive_int(value: str) -> int:
    parsed = int(value)
    if parsed <= 0:
        raise argparse.ArgumentTypeError("il valore deve essere maggiore di zero")
    return parsed


def read_csv(path: Path, executions: int) -> dict[int, Summary]:
    result: dict[int, Summary] = {}
    with path.open(newline="", encoding="utf-8-sig") as stream:
        reader = csv.DictReader(stream)
        fields = set(reader.fieldnames or ())
        required = {"boids", "avg_ms"}
        if not required.issubset(fields):
            missing = ", ".join(sorted(required - fields))
            raise ValueError(f"colonne mancanti: {missing}")

        for line_number, row in enumerate(reader, start=2):
            try:
                boids = int(row["boids"])
                mean = float(row["avg_ms"])
                stddev = float(row.get("stddev_ms") or 0.0)
            except (TypeError, ValueError) as exc:
                raise ValueError(
                    f"valore non numerico alla riga {line_number}"
                ) from exc
            if boids <= 0 or mean <= 0 or stddev < 0:
                raise ValueError(f"valore non valido alla riga {line_number}")
            if boids in result:
                raise ValueError(f"boid duplicati alla riga {line_number}")
            result[boids] = Summary(mean, stddev, executions)

    if not result:
        raise ValueError("il file non contiene dati")
    return result


def merge_summaries(items: Iterable[Summary]) -> Summary:
    """Combina statistiche di piu' file pesandole per le esecuzioni."""
    values = list(items)
    samples = sum(item.samples for item in values)
    mean = sum(item.samples * item.mean for item in values) / samples
    second_moment = sum(
        item.samples * (item.stddev**2 + item.mean**2) for item in values
    ) / samples
    return Summary(mean, math.sqrt(max(0.0, second_moment - mean**2)), samples)


def merge_series(series: Iterable[dict[int, Summary]]) -> dict[int, Summary]:
    grouped: dict[int, list[Summary]] = defaultdict(list)
    for values in series:
        for boids, summary in values.items():
            grouped[boids].append(summary)
    return {boids: merge_summaries(items) for boids, items in grouped.items()}


def load_benchmarks(
    directory: Path,
) -> tuple[
    dict[int, Summary],
    dict[tuple[str, ChunkSize], dict[int, Summary]],
]:
    sequential_files: list[dict[int, Summary]] = []
    parallel_files: dict[
        tuple[str, ChunkSize], list[dict[int, Summary]]
    ] = defaultdict(list)

    for path in sorted(directory.glob("benchmark_*.csv")):
        sequential_match = SEQUENTIAL_RE.fullmatch(path.name)
        parallel_match = PARALLEL_RE.fullmatch(path.name)
        if sequential_match:
            executions = int(sequential_match.group(1))
            target = sequential_files
        elif parallel_match:
            scheduling = parallel_match.group(1).lower()
            raw_chunk_size = parallel_match.group(2).lower()
            chunk_size: ChunkSize = (
                int(raw_chunk_size)
                if raw_chunk_size.isdigit()
                else raw_chunk_size
            )
            executions = int(parallel_match.group(3))
            target = parallel_files[(scheduling, chunk_size)]
        else:
            print(
                f"Avviso: ignoro {path.name}: nome non riconosciuto",
                file=sys.stderr,
            )
            continue
        try:
            target.append(read_csv(path, executions))
        except (OSError, ValueError) as exc:
            raise SystemExit(f"Errore in {path.name}: {exc}") from exc

    if not sequential_files:
        raise SystemExit(
            "Nessun benchmark_sequential_<esecuzioni>.csv trovato in "
            f"{directory}"
        )
    if not parallel_files:
        raise SystemExit(
            "Nessun benchmark_parallel_<scheduling>_<chunk>_<esecuzioni>.csv "
            f"trovato in {directory}"
        )
    return merge_series(sequential_files), {
        configuration: merge_series(files)
        for configuration, files in parallel_files.items()
    }


def plot_series(axis, values: dict[int, Summary], label: str, **style) -> None:
    boids = sorted(values)
    means = [values[count].mean for count in boids]
    deviations = [values[count].stddev for count in boids]
    (line,) = axis.plot(boids, means, marker="o", label=label, **style)
    if any(deviations):
        lower = [
            max(sys.float_info.min, mean - deviation)
            for mean, deviation in zip(means, deviations)
        ]
        upper = [
            mean + deviation for mean, deviation in zip(means, deviations)
        ]
        axis.fill_between(
            boids, lower, upper, color=line.get_color(), alpha=0.14
        )


def create_axes(count: int, title: str):
    columns = min(3, count)
    rows = math.ceil(count / columns)
    figure, axes = plt.subplots(
        rows, columns, figsize=(6.2 * columns, 4.7 * rows), squeeze=False
    )
    figure.suptitle(title, fontsize=15)
    flat_axes = list(axes.flat)
    for unused in flat_axes[count:]:
        unused.remove()
    return figure, flat_axes[:count]


def configure_axis(axis) -> None:
    axis.set_xscale("log")
    axis.set_yscale("log")
    axis.set_xlabel("Numero di boid")
    axis.set_ylabel("Tempo medio per ciclo [ms]")
    axis.grid(True, which="both", alpha=0.28)
    axis.legend()


def chunk_sort_key(chunk: ChunkSize) -> tuple[int, int | str]:
    """Ordina prima i chunk numerici, poi quelli testuali."""
    if isinstance(chunk, int):
        return (0, chunk)
    return (1, chunk.casefold())


def configuration_sort_key(item):
    (schedule, chunk), _ = item
    return (schedule, chunk_sort_key(chunk))


def plot_by_scheduling(sequential, parallel):
    schedules = sorted({schedule for schedule, _ in parallel})
    figure, axes = create_axes(
        len(schedules),
        "Sequenziale vs parallelo - raggruppamento per scheduling",
    )
    for axis, schedule in zip(axes, schedules):
        plot_series(
            axis, sequential, "Sequenziale", color="black", linestyle="--"
        )
        for (candidate, chunk), values in sorted(
            parallel.items(), key=configuration_sort_key
        ):
            if candidate == schedule:
                plot_series(axis, values, f"Parallelo - chunk {chunk}")
        axis.set_title(f"Scheduling: {schedule}")
        configure_axis(axis)
    figure.tight_layout()
    return figure


def plot_by_chunk_size(sequential, parallel):
    chunks = sorted(
        {chunk for _, chunk in parallel}, key=chunk_sort_key
    )
    figure, axes = create_axes(
        len(chunks),
        "Sequenziale vs parallelo - raggruppamento per chunk size",
    )
    for axis, chunk in zip(axes, chunks):
        plot_series(
            axis, sequential, "Sequenziale", color="black", linestyle="--"
        )
        for (schedule, candidate), values in sorted(
            parallel.items(), key=configuration_sort_key
        ):
            if candidate == chunk:
                plot_series(axis, values, f"Parallelo - {schedule}")
        axis.set_title(f"Chunk size: {chunk}")
        configure_axis(axis)
    figure.tight_layout()
    return figure


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dpi", type=positive_int, default=160)
    parser.add_argument(
        "--format", choices=("png", "pdf", "svg"), default="png"
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        help="default: cartella 'plots' accanto allo script",
    )
    parser.add_argument("--show", action="store_true")
    return parser.parse_args()


def main() -> None:
    arguments = parse_arguments()
    script_directory = Path(__file__).resolve().parent
    output_directory = arguments.output_dir or script_directory / "plots"
    output_directory.mkdir(parents=True, exist_ok=True)

    sequential, parallel = load_benchmarks(script_directory)
    figures = {
        "confronto_per_scheduling": plot_by_scheduling(sequential, parallel),
        "confronto_per_chunk_size": plot_by_chunk_size(sequential, parallel),
    }
    for name, figure in figures.items():
        destination = output_directory / f"{name}.{arguments.format}"
        figure.savefig(destination, dpi=arguments.dpi, bbox_inches="tight")
        print(f"Creato: {destination}")

    if arguments.show:
        plt.show()
    else:
        for figure in figures.values():
            plt.close(figure)


if __name__ == "__main__":
    main()
