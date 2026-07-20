#!/usr/bin/env bash
set -Eeuo pipefail

usage() {
  cat <<'EOF'
Usage:
  ./scripts/submit-change.sh "<commit message>" -- <path> [<path> ...]

Stages only the explicit paths, runs repository checks, creates one commit,
and pushes the current feature branch to origin.
EOF
}

if (( $# < 3 )) || [[ "$2" != "--" ]] || [[ -z "${1//[[:space:]]/}" ]]; then
  usage >&2
  exit 2
fi

commit_message="$1"
shift 2
paths=("$@")

repo_root="$(git rev-parse --show-toplevel 2>/dev/null)" || {
  echo "error: run this script inside a Git repository" >&2
  exit 2
}
cd "$repo_root"

current_branch="$(git branch --show-current)"
if [[ -z "$current_branch" ]]; then
  echo "error: detached HEAD; switch to a feature branch first" >&2
  exit 1
fi

case "$current_branch" in
  main|master)
    echo "error: refusing to commit directly to protected branch '$current_branch'" >&2
    echo "Create an agent/<topic> feature branch and run the command again." >&2
    exit 1
    ;;
esac

if ! git diff --cached --quiet; then
  echo "error: the index already contains staged changes" >&2
  echo "Commit or unstage them before using this helper." >&2
  exit 1
fi

for path in "${paths[@]}"; do
  if [[ "$path" == "." || "$path" == /* || "$path" == :* ]] || \
    [[ "$path" == ".." || "$path" == ../* || "$path" == */.. || "$path" == */../* ]]; then
    echo "error: path must stay inside the repository: $path" >&2
    exit 2
  fi
done

git add -- "${paths[@]}"

if git diff --cached --quiet; then
  echo "error: the selected paths contain no changes" >&2
  exit 1
fi

commit_created=0
cleanup_on_error() {
  status=$?
  if (( status != 0 )); then
    if (( commit_created == 0 )); then
      echo "Submission stopped. Review the staged changes with: git diff --cached" >&2
    else
      echo "Commit was created locally, but the push failed. Retry with:" >&2
      echo "  git push -u origin $current_branch" >&2
    fi
  fi
  exit "$status"
}
trap cleanup_on_error ERR

"$repo_root/scripts/check-repository.sh" --staged

echo "Staged change summary:"
git diff --cached --stat

git commit -m "$commit_message"
commit_created=1
git push -u origin "$current_branch"

trap - ERR
echo "Published commit $(git rev-parse --short HEAD) to origin/$current_branch."
