#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# One XT16 PointCloud2 sample is approximately 812 KiB on this deployment.
# CycloneDDS explicitly requests a 16 MiB reader buffer, so the host maximum
# must permit that request.  The kernel default may remain smaller: raising it
# would enlarge every UDP socket, while the measured navigation fix only
# requires the DDS readers to opt into the larger maximum.
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

if (( rmem_max < required_bytes )); then
  cat >&2 <<EOF
ERROR: Go2 DDS UDP receive buffers are too small for sustained XT16 point clouds.
Apply the repository's persistent host setting before the no-motion test:
  sudo env GO2_DDS_HOST_TUNING_CONFIRM=I_AM_CONFIGURING_GO2_DDS_HOST \
    ${SCRIPT_DIR}/install_go2_dds_receive_buffer.sh --apply
Or apply the equivalent setting temporarily:
  sudo sysctl -w net.core.rmem_max=${required_bytes}
No ROS process or physical motion output was started.
EOF
  exit 1
fi

printf 'GO2_DDS_RECEIVE_BUFFER_CHECK=PASS\n'
