#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPOSITORY_ROOT="$(cd -- "${SCRIPT_DIR}/../.." && pwd)"
REPLAY_RUN="${REPOSITORY_ROOT}/data/data02_online_successive_v2/data02-online-successive-extension-v2"
episode="episode_000061"
transition="4"
duration="12"
hold_option="--no-hold"
rgb_option="--show-rgb"

usage() {
    echo "Usage: $0 [--episode ID] [--transition N] [--duration 10..15] [--hold|--no-hold] [--show-rgb|--no-show-rgb]"
}

while (($#)); do
    case "$1" in
        --episode)
            [[ $# -ge 2 ]] || { echo "--episode requires a value" >&2; exit 2; }
            episode="$2"
            shift 2
            ;;
        --transition)
            [[ $# -ge 2 ]] || { echo "--transition requires a value" >&2; exit 2; }
            transition="$2"
            shift 2
            ;;
        --duration)
            [[ $# -ge 2 ]] || { echo "--duration requires a value" >&2; exit 2; }
            duration="$2"
            shift 2
            ;;
        --hold|--no-hold)
            hold_option="$1"
            shift
            ;;
        --show-rgb|--no-show-rgb)
            rgb_option="$1"
            shift
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            echo "Unknown option: $1" >&2
            usage >&2
            exit 2
            ;;
    esac
done

exec "${SCRIPT_DIR}/run_data02_v2_online_successive.sh" \
    --replay-run "${REPLAY_RUN}" \
    --episode "${episode}" \
    --transition "${transition}" \
    --duration "${duration}" \
    --demo --gui "${hold_option}" "${rgb_option}"
