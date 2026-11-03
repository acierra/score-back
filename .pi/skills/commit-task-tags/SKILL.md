---
name: commit-task-tags
description: Manage kanban task tags (TG-<NUMBER> #<state>) in git commit messages. Use when creating, squashing, or rebasing commits to ensure task state trailers are correct.
---

# Commit Task Tags

This project tracks kanban board tasks inside git commit messages using trailer
lines in the format:

```
TG-<NUMBER> #<state>
```

Multiple trailers may appear in a single commit (one per related task). Trailers
go at the bottom of the commit body, separated from the prose by a blank line.

## States

| State | Meaning |
|-------|---------|
| `new` | Task created on the board; no work started yet. |
| `ready` | Task is defined and ready to be picked up. |
| `in-progress` | This commit contributes to the task, but the full task/epic is **not** yet implemented. Use for every intermediate commit. |
| `ready-for-test` | The **full implementation** of the task/epic is completed across all its commits. Place on the final commit of the task. |
| `done` | Property of the **full task**, not of an individual commit. Reserved for final board confirmation after testing. **Never** use `#done` in a code commit. |

### Key rules

1. **`#done` is not for commits.** A commit can at most mark a task
   `#ready-for-test`. The `#done` state is set on the kanban board after
   verification, not in git.

2. **Multi-commit epics use `#in-progress` then `#ready-for-test`.** Every
   intermediate commit for a task carries `#in-progress`. Only the final commit
   that completes the implementation switches to `#ready-for-test`.

3. **Single-commit tasks use `#ready-for-test`.** If a task is fully implemented
   in one commit, that commit carries `#ready-for-test` (not `#done`).

4. **One task may span many commits.** Add the same `TG-<N>` trailer to every
   commit that touches that task's work, updating the state as appropriate.

## Example commit

```
feat(cloud_point): stereo rectification and point cloud pipeline

- StereoRectifier wrapping cv::stereoRectify
- PointCloudBuilder with depth filtering
- E2E synthetic-scene test

TG-9 #ready-for-test
TG-2 #in-progress
```

## Adding tags to existing commits

### During interactive rebase (recommended)

Use `git rebase -i` with `edit` stops, then amend each commit:

```bash
# Strip old TG lines and append new ones in one command:
git log -1 --format=%B \
  | sed '/^TG-[0-9]/d' \
  | { cat; printf '\nTG-<N> #<state>\n'; } \
  | git commit --amend --no-verify -F -
git rebase --continue
```

> **Avoid `reword`** with a shared `GIT_EDITOR` script — it can shift tags by one
> commit. Use `edit` stops with explicit `git commit --amend` instead.

### Helper script

A reusable script lives at `scripts/tag-commit.sh` in this skill directory.

```bash
# At a rebase edit stop, add or replace tags:
../scripts/tag-commit.sh "TG-5 #ready-for-test" "TG-2 #in-progress"
git rebase --continue
```

The script strips any existing `TG-*` lines from the current commit message,
appends the supplied trailers, and amends the commit.

## Squashing commits

When squashing many commits into fewer logical commits, reassign task tags to
the resulting squashed commits following the same state rules. After squashing:

1. Identify which tasks each squashed commit covers.
2. Mark intermediate squash commits `#in-progress` for tasks that continue in
   later squash commits.
3. Mark the final squash commit for a task `#ready-for-test`.

Use `git reset --hard <group-end>` + `git reset --soft <prev-group-commit>` to
snapshot each group's tree, then `git commit` with the appropriate trailers.