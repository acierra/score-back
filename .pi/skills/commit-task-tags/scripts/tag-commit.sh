#!/bin/sh
# tag-commit.sh — Add or replace TG-<NUMBER> #<state> trailers on the current commit.
#
# Usage (at a rebase edit stop or any HEAD you want to amend):
#   ./tag-commit.sh "TG-5 #ready-for-test" "TG-2 #in-progress"
#
# Strips existing TG-* lines, appends the given trailers, and amends the commit.

set -eu

if [ "$#" -eq 0 ]; then
  echo "Usage: $0 \"TG-<N> #<state>\" [\"TG-<N> #<state>\" ...]" >&2
  exit 1
fi

# Build the trailer block from arguments.
trailers=""
for tag in "$@"; do
  trailers="${trailers}${tag}"$'\n'
done

# Strip existing TG-* lines, append new trailers, amend commit.
git log -1 --format=%B \
  | sed '/^TG-[0-9]/d' \
  | { cat; printf '\n%s' "$trailers"; } \
  | git commit --amend --no-verify -F -