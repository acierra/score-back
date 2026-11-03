#!/usr/bin/env bash
# Publish the openwiki/ pages to the Gitea wiki of this repository.
#
# Usage:
#   scripts/sync_gitea_wiki.sh [--dry-run] [--prune]
#
#   --dry-run  show what would change in the wiki, do not commit or push
#   --prune    also delete wiki pages that no longer exist in openwiki/
#              (off by default so hand-written pages such as _Sidebar.md stay)
#
# Environment (defaults in brackets):
#   REPO_URL    web URL of the repository      [origin remote without .git]
#   WIKI_URL    git URL of the wiki            [$REPO_URL.wiki.git]
#   SOURCE_DIR  directory with the pages       [openwiki]
#   HOME_PAGE   source page that becomes Home  [index.md]
#   BRANCH      branch that ../ links point to [master]
#
# The pages are adapted to the wiki on the way:
#   - $HOME_PAGE is published as Home.md, the wiki landing page;
#   - links between pages lose their .md suffix (wiki pages have none);
#   - links that leave openwiki/ (../docs/...) point at the repository
#     ($REPO_URL/src/branch/$BRANCH/..., images at .../raw/branch/...).
# INSTRUCTIONS.md (the OpenWiki generator prompt) is not published.
#
# Gitea creates the wiki repository together with its first page, so create
# one page in the web UI before the first run.

set -euo pipefail

dry_run=false
prune=false
for arg in "$@"; do
  case "$arg" in
    --dry-run) dry_run=true ;;
    --prune) prune=true ;;
    -h | --help)
      sed -n '2,/^$/s/^# \{0,1\}//p' "$0"
      exit 0
      ;;
    *)
      echo "Unknown option: $arg" >&2
      exit 1
      ;;
  esac
done

cd "$(git -C "$(dirname "$0")" rev-parse --show-toplevel)"

if [[ -z "${REPO_URL:-}" ]]; then
  REPO_URL="$(git remote get-url origin)"
  REPO_URL="${REPO_URL%.git}"
fi
WIKI_URL="${WIKI_URL:-$REPO_URL.wiki.git}"
SOURCE_DIR="${SOURCE_DIR:-openwiki}"
HOME_PAGE="${HOME_PAGE:-index.md}"
BRANCH="${BRANCH:-master}"
home_name="${HOME_PAGE%.md}"

if [[ ! -d "$SOURCE_DIR" ]]; then
  echo "Source directory not found: $SOURCE_DIR" >&2
  exit 1
fi

wiki_dir="$(mktemp -d)"
trap 'rm -rf "$wiki_dir"' EXIT

if ! git clone --quiet "$WIKI_URL" "$wiki_dir"; then
  echo "Could not clone $WIKI_URL" >&2
  echo "Check your access and that the wiki has at least one page." >&2
  exit 1
fi

if $prune; then
  find "$wiki_dir" -path "$wiki_dir/.git" -prune -o -type f -name '*.md' \
    -exec rm -f {} +
fi

# Order matters: image links first, then other ../ links, then page links.
rewrite_links() {
  sed -E \
    -e "s#!\[([^]]*)\]\(\.\./([^)]*)\)#![\1]($REPO_URL/raw/branch/$BRANCH/\2)#g" \
    -e "s#\]\(\.\./([^)]*)\)#]($REPO_URL/src/branch/$BRANCH/\1)#g" \
    -e "s#\]\((\./)?$home_name\.md(\#[^)]*)?\)#](Home\2)#g" \
    -e "s#\]\((\./)?([A-Za-z0-9_./-]+)\.md(\#[^)]*)?\)#](\2\3)#g"
}

find "$SOURCE_DIR" -type f -name '*.md' ! -name 'INSTRUCTIONS.md' -print0 |
  while IFS= read -r -d '' file; do
    relative="${file#"$SOURCE_DIR"/}"
    [[ "$relative" == "$HOME_PAGE" ]] && relative="Home.md"
    mkdir -p "$wiki_dir/$(dirname "$relative")"
    rewrite_links <"$file" >"$wiki_dir/$relative"
  done

git -C "$wiki_dir" add --all
if git -C "$wiki_dir" diff --cached --quiet; then
  echo "Wiki is already up to date."
  exit 0
fi

git -C "$wiki_dir" diff --cached --stat
if $dry_run; then
  echo "Dry run: nothing committed or pushed."
  exit 0
fi

source_rev="$(git rev-parse --short HEAD)"
git -C "$wiki_dir" commit --quiet \
  -m "Sync wiki from $SOURCE_DIR/ at $source_rev"
git -C "$wiki_dir" push --quiet
echo "Wiki updated: $REPO_URL/wiki"
