#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
CHECKER="${SCRIPT_DIR}/check_go2_dds_receive_buffers.sh"
SOURCE_CONFIG="${WS_ROOT}/config/sysctl.d/90-go2-dds-receive-buffer.conf"
TARGET_CONFIG="/etc/sysctl.d/90-go2-dds-receive-buffer.conf"
CONFIRM_PHRASE="I_AM_CONFIGURING_GO2_DDS_HOST"

usage() {
  cat <<'EOF'
Usage:
  scripts/install_go2_dds_receive_buffer.sh --check
  scripts/install_go2_dds_receive_buffer.sh --print-config
  sudo env GO2_DDS_HOST_TUNING_CONFIRM=I_AM_CONFIGURING_GO2_DDS_HOST \
    scripts/install_go2_dds_receive_buffer.sh --apply

--check        Read-only validation of the current host receive-buffer limit.
--print-config Print the repository-owned persistent sysctl configuration.
--apply        Install and load that configuration, then validate it.

This script does not start ROS, Docker, navigation, or physical motion output.
EOF
}

die() {
  printf 'ERROR: %s\n' "$*" >&2
  exit 1
}

mode="${1:---check}"
(( $# <= 1 )) || { usage >&2; exit 2; }

[[ -x "${CHECKER}" ]] || die "DDS receive-buffer checker is not executable: ${CHECKER}"
[[ -r "${SOURCE_CONFIG}" ]] || die "Missing sysctl source configuration: ${SOURCE_CONFIG}"

case "${mode}" in
  --check)
    exec "${CHECKER}"
    ;;
  --print-config)
    cat "${SOURCE_CONFIG}"
    ;;
  --apply)
    [[ "${GO2_DDS_HOST_TUNING_CONFIRM:-}" == "${CONFIRM_PHRASE}" ]] || \
      die "--apply requires GO2_DDS_HOST_TUNING_CONFIRM=${CONFIRM_PHRASE}"
    (( EUID == 0 )) || \
      die "--apply must run as root; use the sudo command shown by --help."
    install -D -m 0644 "${SOURCE_CONFIG}" "${TARGET_CONFIG}"
    sysctl -p "${TARGET_CONFIG}"
    "${CHECKER}"
    printf 'GO2_DDS_SYSCTL_CONFIG=%s\n' "${TARGET_CONFIG}"
    printf 'GO2_DDS_HOST_TUNING=APPLIED\n'
    ;;
  -h|--help|help)
    usage
    ;;
  *)
    printf 'ERROR: unknown argument: %s\n' "${mode}" >&2
    usage >&2
    exit 2
    ;;
esac
