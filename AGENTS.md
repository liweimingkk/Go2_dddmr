# Repository Instructions

## Scope

- These instructions apply to the entire repository.
- A more deeply nested `AGENTS.md` adds directory-specific instructions. Follow both files, with the more specific instruction taking precedence when they conflict.
- Preserve user-authored changes and keep every task limited to the files needed for that task.

## Required Git Workflow

- Start work by checking `git status --short --branch` and the current branch.
- Do not mix unrelated working-tree changes into a commit. Stage explicit paths instead of using `git add -A` in a mixed worktree.
- After completing a coherent user-requested change, inspect the diff and run the relevant checks before committing.
- Unless the user asks for local-only changes, commit completed changes with a concise message that explains the purpose, then push the current feature branch.
- When starting from `main` or `master`, create a descriptive `agent/<topic>` branch and open a pull request instead of pushing directly to the default branch.
- Prefer the repository helper for the final commit and push:

  ```bash
  ./scripts/submit-change.sh "<concise commit message>" -- <path> [<path> ...]
  ```

- Never commit credentials, private keys, tokens, runtime logs, ROS bags, generated build trees, or unrelated large artifacts.
- Never force-push or rewrite published history unless the user explicitly requests it and the consequences have been explained.
- If checks or the push fail, keep the local work intact and report the exact failure. Do not claim that a change was published until the remote push succeeds.

## Validation

- Run `./scripts/check-repository.sh` while developing and `./scripts/check-repository.sh --staged` before committing.
- Run any additional tests relevant to the files changed. The repository check is a baseline, not a replacement for package-specific tests.
- Changes under `dddmr_navigation/` must also follow `dddmr_navigation/AGENTS.md`, especially its live-robot and motion-safety boundaries.

## Learning-First Development

- Treat learning material as part of the deliverable for substantial changes. A substantial change includes a new feature, a non-trivial bug fix, or a meaningful algorithm, architecture, interface, data-flow, or key-configuration change.
- Before implementing a substantial change, give the user a concise Chinese learning brief that explains the problem, the current behavior or data flow, the core principle, the proposed approach and alternatives, the expected edit points, and the validation method. Continue without a mandatory pause unless a high-impact product or engineering choice needs the user's decision.
- After implementing a substantial change, create or update `docs/learning/YYYY-MM-DD-<topic>.md` and link it in the final handoff. Use `docs/learning/README.md` as the required structure.
- Connect explanations to the actual repository: name the important entry points, functions, topics, interfaces, or configuration keys and explain how data or control moves through them. Do not replace explanation with large pasted code blocks.
- Clearly separate verified facts and test results from assumptions, hypotheses, and work that still needs live or manual validation.
- Include employment-oriented material without exaggeration: transferable concepts, likely interview questions, a small hands-on exercise, and a truthful resume-project bullet that the user can adapt only after understanding the work.
- Spelling, comments, formatting, generated-file refreshes, and simple constant or text changes do not require a standalone learning document. Their final handoff must still state what changed, why it changed, and how it was checked.
- Apply this workflow to future work; do not backfill learning documents for historical changes unless the user asks.

## Handoff

- Report the branch, commit, pushed remote, and validation performed.
- For substantial changes, link the learning document and briefly state which concepts it teaches.
- Clearly identify anything that was not tested or any manual follow-up still required.
