#!/usr/bin/env bash
set -Eeuo pipefail

# One XT16 PointCloud2 sample is approximately 650 KiB on this deployment.
# Require ample headroom so a DDS reader cannot start with a receive buffer
# smaller than one sample and silently degrade after a short burst.
required_bytes=16777216
rmem_max_path="/proc/sys/net/core/rmem_max"
rmem_default_path="/proc/sys/net/core/rmem_default"

while (( $# > 0 )); do
  case "$1" in
    --rmem-max-path)
      (( $# >= 2 )) || { echo "ERROR: --rmem-max-path requires a value." >&2; exit 2; }
      rmem_max_path="$2"
      shift 2
      ;;
    --rmem-default-path)
      (( $# >= 2 )) || { echo "ERROR: --rmem-default-path requires a value." >&2; exit 2; }
      rmem_default_path="$2"
      shift 2
      ;;
    *)
      printf 'ERROR: unknown argument: %s\n' "$1" >&2
      exit 2
      ;;
  esac
done

read_positive_integer() {
  local name="$1"
  local path="$2"
  local value=""

  [[ -r "${path}" ]] || {
    printf 'ERROR: cannot read %s from %s\n' "${name}" "${path}" >&2
    return 1
  }
  IFS= read -r value <"${path}" || true
  [[ "${value}" =~ ^[1-9][0-9]*$ ]] || {
    printf 'ERROR: %s in %s is not a positive integer: %s\n' \
      "${name}" "${path}" "${value:-<empty>}" >&2
    return 1
  }
  printf '%s\n' "${value}"
}

rmem_max="$(read_positive_integer net.core.rmem_max "${rmem_max_path}")" || exit 1
rmem_default="$(read_positive_integer net.core.rmem_default "${rmem_default_path}")" || exit 1

printf 'GO2_DDS_REQUIRED_RMEM_BYTES=%s\n' "${required_bytes}"
printf 'GO2_DDS_RMEM_MAX_BYTES=%s\n' "${rmem_max}"
printf 'GO2_DDS_RMEM_DEFAULT_BYTES=%s\n' "${rmem_default}"

if (( rmem_max < required_bytes || rmem_default < required_bytes )); then
  cat >&2 <<EOF
ERROR: Go2 DDS UDP receive buffers are too small for sustained XT16 point clouds.
Run these commands on the host before the no-motion test:
  sudo sysctl -w net.core.rmem_max=${required_bytes}
  sudo sysctl -w net.core.rmem_default=${required_bytes}
No ROS process or physical motion output was started.
EOF
  exit 1
fi

printf 'GO2_DDS_RECEIVE_BUFFER_CHECK=PASS\n'
