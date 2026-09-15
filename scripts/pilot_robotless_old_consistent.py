#!/usr/bin/env python3
"""Freeze or analyze the five-case OLD-consistent observation pilot."""
import argparse
from pathlib import Path
import robotless_old_consistent_artifacts as a

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mode', choices=['freeze', 'analyze'])
    parser.add_argument('run_directory', type=Path)
    parser.add_argument('--config', type=Path, default=Path('configs/robotless_old_consistent_observation.yaml'))
    args = parser.parse_args()
    result = a.freeze(args.config.resolve(), args.run_directory.resolve()) if args.mode == 'freeze' else a.analyze(args.run_directory.resolve())
    print({k: result[k] for k in ('case_count','valid_count','invalid_count') if k in result})
