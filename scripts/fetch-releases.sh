#!/usr/bin/env bash
set -euo pipefail

# Mirrors each GitHub Release's BMCVFirmware.bin into web/firmware/<tag>/, and
# writes web/firmware/index.json (newest first). The updater page reads that
# to offer a version dropdown, and fetches the binary from there rather than
# from the release directly: GitHub's release-asset CDN does not send
# Access-Control-Allow-Origin, so a browser fetch() of it is blocked no matter
# which of GitHub's asset URL forms is used - same-origin, on the page's own
# Pages deploy, sidesteps that instead of working around it.
#
# Needs `gh` authenticated (GH_TOKEN) and `jq`; both are on the GitHub-hosted
# runner image already. Run from BMCVFirmware/, as `just` and CI both do.

DEST="${1:-web/firmware}"
REPO="${GH_REPO:-Bemeier/bmcv}"

rm -rf "$DEST"
mkdir -p "$DEST"

manifest="[]"
while IFS= read -r tag; do
  [ -n "$tag" ] || continue
  asset_dir="$DEST/$tag"
  mkdir -p "$asset_dir"
  if gh release download "$tag" --repo "$REPO" \
       --pattern 'BMCVFirmware.bin' --dir "$asset_dir" --clobber 2>/dev/null; then
    size=$(stat -c%s "$asset_dir/BMCVFirmware.bin")
    date=$(gh release view "$tag" --repo "$REPO" --json publishedAt --jq .publishedAt)
    manifest=$(jq --arg tag "$tag" --arg date "$date" --argjson size "$size" \
      '. + [{tag: $tag, date: $date, size: $size, path: ($tag + "/BMCVFirmware.bin")}]' \
      <<<"$manifest")
  else
    echo "No BMCVFirmware.bin on $tag - skipping." >&2
    rmdir "$asset_dir" 2>/dev/null || true
  fi
done < <(gh release list --repo "$REPO" --exclude-drafts --exclude-pre-releases --json tagName --jq '.[].tagName')

jq . <<<"$manifest" > "$DEST/index.json"
echo "Wrote $DEST/index.json with $(jq length <<<"$manifest") version(s)."
