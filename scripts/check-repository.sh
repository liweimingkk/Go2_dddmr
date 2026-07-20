#!/usr/bin/env bash
set -Eeuo pipefail

usage() {
  cat <<'EOF'
Usage:
  ./scripts/check-repository.sh
  ./scripts/check-repository.sh --staged
  ./scripts/check-repository.sh --range <base-revision> <head-revision>

Checks shell syntax for repository scripts and inspects changed files for
whitespace errors, merge markers, sensitive filenames, and oversized files.
Without an option, staged changes are checked when present; otherwise working
tree changes relative to HEAD are checked.
EOF
}

repo_root="$(git rev-parse --show-toplevel 2>/dev/null)" || {
  echo "error: run this script inside a Git repository" >&2
  exit 2
}
cd "$repo_root"

mode="auto"
base_revision=""
head_revision=""

case "${1:-}" in
  "")
    ;;
  --staged)
    mode="staged"
    shift
    ;;
  --range)
    if (( $# != 3 )); then
      usage >&2
      exit 2
    fi
    mode="range"
    base_revision="$2"
    head_revision="$3"
    shift 3
    ;;
  -h|--help)
    usage
    exit 0
    ;;
  *)
    usage >&2
    exit 2
    ;;
esac

if (( $# != 0 )); then
  usage >&2
  exit 2
fi

if [[ "$mode" == "auto" ]]; then
  if git diff --cached --quiet; then
    mode="working"
  else
    mode="staged"
  fi
fi

declare -a changed_files=()
case "$mode" in
  staged)
    mapfile -d '' -t changed_files < <(
      git diff --cached --name-only --diff-filter=ACMR -z
    )
    git diff --cached --check
    ;;
  working)
    mapfile -d '' -t changed_files < <(
      {
        git diff --name-only --diff-filter=ACMR -z HEAD
        git ls-files --others --exclude-standard -z
      } | sort -zu
    )
    git diff --check HEAD
    ;;
  range)
    git rev-parse --verify "${base_revision}^{commit}" >/dev/null
    git rev-parse --verify "${head_revision}^{commit}" >/dev/null
    mapfile -d '' -t changed_files < <(
      git diff --name-only --diff-filter=ACMR -z \
        "$base_revision" "$head_revision"
    )
    git diff --check "$base_revision" "$head_revision"
    ;;
esac

declare -a shell_files=()
mapfile -d '' -t shell_files < <(
  git ls-files -co --exclude-standard -z -- '*.sh' '*.bash'
)

for path in "${shell_files[@]}"; do
  [[ -f "$path" ]] || continue
  bash -n "$path"
done

failure=0
max_file_size=$((50 * 1024 * 1024))

for path in "${changed_files[@]}"; do
  [[ -f "$path" ]] || continue

  case "$path" in
    *.env.example|*.example.env|*.pem.example|*.key.example)
      ;;
    .env|*/.env|.env.*|*/.env.*|*.pem|*.p12|*.pfx|*.key|*/id_rsa|*/id_ed25519|*/credentials.json)
      echo "error: potentially sensitive file must not be committed: $path" >&2
      failure=1
      ;;
  esac

  if grep -nE '^(<{7}|>{7})( |$)' -- "$path" >/dev/null; then
    echo "error: unresolved merge marker found in: $path" >&2
    failure=1
  fi

  file_size="$(stat -c '%s' -- "$path")"
  if (( file_size > max_file_size )); then
    echo "error: changed file exceeds 50 MiB: $path" >&2
    failure=1
  fi
done

if (( failure != 0 )); then
  exit 1
fi

echo "Repository checks passed (${#changed_files[@]} changed files inspected)."
