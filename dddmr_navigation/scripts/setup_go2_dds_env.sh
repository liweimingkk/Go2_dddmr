#!/usr/bin/env bash
# Source this file before host-side or Docker-side Go2 ROS 2 reads.

if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
  echo "This script must be sourced, not executed:" >&2
  echo "  source ${BASH_SOURCE[0]}" >&2
  exit 2
fi

GO2_DDS_IP="${GO2_DDS_IP:-192.168.123.18}"

detect_go2_net_iface() {
  if [[ -n "${GO2_NET_IFACE:-}" ]]; then
    printf '%s\n' "${GO2_NET_IFACE}"
    return
  fi

  local route_iface
  route_iface="$(ip route get "${GO2_DDS_IP}" 2>/dev/null | awk '
    {
      for (i = 1; i <= NF; i++) {
        if ($i == "dev" && (i + 1) <= NF) {
          print $(i + 1)
          exit
        }
      }
    }
  ')"
  if [[ -n "${route_iface}" ]]; then
    printf '%s\n' "${route_iface}"
    return
  fi

  printf '%s\n' "enp5s0"
}

GO2_NET_IFACE="$(detect_go2_net_iface)"
export GO2_DDS_IP
export GO2_NET_IFACE
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"

GO2_DDS_RCVBUF_MAX="${GO2_DDS_RCVBUF_MAX:-16MiB}"
GO2_DDS_SNDBUF_MAX="${GO2_DDS_SNDBUF_MAX:-16MiB}"
GO2_DDS_FRAGMENT_SIZE="${GO2_DDS_FRAGMENT_SIZE:-65000B}"
GO2_DDS_ALLOW_MULTICAST="${GO2_DDS_ALLOW_MULTICAST:-true}"
export GO2_DDS_RCVBUF_MAX
export GO2_DDS_SNDBUF_MAX
export GO2_DDS_FRAGMENT_SIZE
export GO2_DDS_ALLOW_MULTICAST

export CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces>
  <NetworkInterface name=\"${GO2_NET_IFACE}\" priority=\"default\" multicast=\"${GO2_DDS_ALLOW_MULTICAST}\" />
</Interfaces><AllowMulticast>${GO2_DDS_ALLOW_MULTICAST}</AllowMulticast><MaxMessageSize>65500B</MaxMessageSize><FragmentSize>${GO2_DDS_FRAGMENT_SIZE}</FragmentSize></General>
<Internal><SocketReceiveBufferSize min=\"default\" max=\"${GO2_DDS_RCVBUF_MAX}\" /><SocketSendBufferSize min=\"default\" max=\"${GO2_DDS_SNDBUF_MAX}\" /></Internal>
</Domain></CycloneDDS>"
