#!/usr/bin/env bash
# Create / use the HPP planning container for soarm_tamp.
#
# Planning needs pyhpp, which exists only inside the hpp-agimus image;
# the servos hang off the host's USB. This script owns the container half.
#
#   ./scripts/hpp_container.sh up        create (or recreate) the container
#   ./scripts/hpp_container.sh plan ...  run soarm_tamp.plan inside it
#   ./scripts/hpp_container.sh tcp ...   run soarm_tamp.plan_tcp inside it
#   ./scripts/hpp_container.sh replay ... replay a manifest in the viser viewer
#   ./scripts/hpp_container.sh shell     interactive shell with HPP sourced
#   ./scripts/hpp_container.sh exec ...  run an arbitrary command inside it
#
# This deliberately uses its OWN container name rather than the existing
# hpp-agimus-arm64 one: that container predates soarm-ws and has no mount
# for it, and adding a mount to a container means recreating it, which
# would throw away its writable layer. Nothing here touches it.
set -euo pipefail

NAME="${SOARM_TAMP_CONTAINER:-hpp-soarm-tamp}"
IMAGE="${SOARM_TAMP_IMAGE:-hpp-agimus:arm64}"
HOST_HPP="${HOST_HPP_DIR:-$HOME/devel/hpp}"
HOST_LONGTAMP="${LONG_TAMP_DIR:-$HOME/Develop/agimus-ws/long-tamp}"
HOST_SOARM="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
CUSER="thanhndv212"
CHOME="/home/${CUSER}"

# PYTHONPATH additions: long_tamp ships as a source tree, and soarm_tamp is
# this package. The image's config.sh wires up HPP itself but knows about
# neither.
ENVSETUP="source ${CHOME}/devel/hpp/config.sh >/dev/null 2>&1; \
export PYTHONPATH=${CHOME}/devel/hpp/src/long_tamp/src:${CHOME}/devel/soarm-ws/soarm_tamp:\$PYTHONPATH"

up() {
  docker rm -f "$NAME" >/dev/null 2>&1 || true
  docker run -d --name "$NAME" --platform linux/arm64 --network host \
    -v "$HOST_HPP:${CHOME}/devel/hpp:rw" \
    -v hpp-arm64-install:"${CHOME}/devel/hpp/install" \
    -v "$HOST_LONGTAMP:${CHOME}/devel/hpp/src/long_tamp" \
    -v "$HOST_SOARM:${CHOME}/devel/soarm-ws" \
    --env HOME="$CHOME" \
    "$IMAGE" sleep infinity >/dev/null
  echo "container '$NAME' up"
}

ensure() {
  if ! docker ps --format '{{.Names}}' | grep -qx "$NAME"; then
    docker start "$NAME" >/dev/null 2>&1 || up
  fi
}

case "${1:-}" in
  up)    up ;;
  shell) ensure; shift; docker exec -it "$NAME" bash -c "${ENVSETUP}; exec bash" ;;
  replay) ensure; shift
         docker exec -i "$NAME" bash -c \
           "${ENVSETUP}; cd ${CHOME}/devel/soarm-ws/soarm_tamp && python3 -u -m soarm_tamp.replay $*" ;;
  tcp)   ensure; shift
         docker exec -i "$NAME" bash -c \
           "${ENVSETUP}; cd ${CHOME}/devel/soarm-ws/soarm_tamp && python3 -u -m soarm_tamp.plan_tcp $*" ;;
  plan)  ensure; shift
         docker exec -i "$NAME" bash -c \
           "${ENVSETUP}; cd ${CHOME}/devel/soarm-ws/soarm_tamp && python3 -u -m soarm_tamp.plan $*" ;;
  exec)  ensure; shift; docker exec -i "$NAME" bash -c "${ENVSETUP}; $*" ;;
  *)     sed -n '2,14p' "$0"; exit 1 ;;
esac
