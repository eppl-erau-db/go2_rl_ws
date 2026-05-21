#!/usr/bin/env python3
"""Find rosbag2 directories whose names contain a requested date string."""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Iterable, List


DEFAULT_DATE_SUBSTRING = '5_11'


def _candidate_paths(root: Path, recursive: bool) -> Iterable[Path]:
    return root.rglob('*') if recursive else root.iterdir()


def find_dated_bags(
    bag_root: str | Path,
    date_substring: str = DEFAULT_DATE_SUBSTRING,
    recursive: bool = False,
) -> List[Path]:
    """Return sorted rosbag2 directories below *bag_root* containing *date_substring*."""
    root = Path(bag_root).expanduser()
    if not root.exists():
        return []

    bags = [
        path.resolve()
        for path in _candidate_paths(root, recursive)
        if path.is_dir()
        and date_substring in path.name
        and (path / 'metadata.yaml').is_file()
    ]
    return sorted(bags, key=lambda path: path.name)


def choose_bag(
    bag_root: str | Path,
    date_substring: str = DEFAULT_DATE_SUBSTRING,
    bag_index: int = -1,
    recursive: bool = False,
) -> Path:
    """Choose one matching bag by index, using -1 for the newest lexical match."""
    bags = find_dated_bags(bag_root, date_substring, recursive)
    if not bags:
        raise FileNotFoundError(
            f'No rosbag2 directories containing {date_substring!r} found under '
            f'{Path(bag_root).expanduser()}'
        )

    try:
        return bags[bag_index]
    except IndexError as exc:
        raise IndexError(
            f'bag_index {bag_index} is out of range for {len(bags)} matching bags'
        ) from exc


def main() -> None:
    parser = argparse.ArgumentParser(
        description='List rosbag2 directories matching a date substring.',
    )
    parser.add_argument('--bag-root', default=str(Path.home()))
    parser.add_argument('--date', default=DEFAULT_DATE_SUBSTRING)
    parser.add_argument('--recursive', action='store_true')
    args = parser.parse_args()

    for bag in find_dated_bags(args.bag_root, args.date, args.recursive):
        print(bag)


if __name__ == '__main__':
    main()
