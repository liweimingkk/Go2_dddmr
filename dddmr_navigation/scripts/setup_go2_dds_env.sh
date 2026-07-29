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
  if [[ -n "${route_iface}" && "${route_iface}" != "lo" ]]; then
    printf '%s\n' "${route_iface}"
    return
  fi

  # `ip route get` reports `lo` when GO2_DDS_IP is an address owned by this
  # host (the Orin default is 192.168.123.18). CycloneDDS must bind the real
  # interface that owns that address, otherwise discovery succeeds only on
  # loopback and live XT16 samples remain invisible.
  local address_iface
  address_iface="$(ip -o -4 address show 2>/dev/null | awk \
    -v target="${GO2_DDS_IP}" '
    {
      split($4, address, "/")
      if (address[1] == target && $2 != "lo") {
        print $2
        exit
      }
    }
  ')"
  if [[ -n "${address_iface}" ]]; then
    printf '%s\n' "${address_iface}"
    return
  fi

  if [[ -n "${route_iface}" ]]; then
    printf '%s\n' "${route_iface}"
    return
  fi

  printf '%s\n' "enp5s0"
}

GO2_NET_IFACE="$(detect_go2_net_iface)"
GO2_DDS_EXTRA_IFACES="${GO2_DDS_EXTRA_IFACES:-}"
GO2_DDS_PEERS="${GO2_DDS_PEERS:-}"
GO2_DDS_PARTICIPANT_INDEX="${GO2_DDS_PARTICIPANT_INDEX:-auto}"
GO2_DDS_MAX_AUTO_PARTICIPANT_INDEX="${GO2_DDS_MAX_AUTO_PARTICIPANT_INDEX:-120}"
GO2_DDS_PRIMARY_ADDRESS="${GO2_DDS_PRIMARY_ADDRESS:-}"
GO2_DDS_ROBOT_TOPIC_PATTERNS="${GO2_DDS_ROBOT_TOPIC_PATTERNS:-rt/utlidar/* rt/uslam/*}"
export GO2_DDS_IP
export GO2_NET_IFACE
export GO2_DDS_EXTRA_IFACES
export GO2_DDS_PEERS
export GO2_DDS_PARTICIPANT_INDEX
export GO2_DDS_MAX_AUTO_PARTICIPANT_INDEX
export GO2_DDS_PRIMARY_ADDRESS
export GO2_DDS_ROBOT_TOPIC_PATTERNS
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"

GO2_DDS_RCVBUF_MAX="${GO2_DDS_RCVBUF_MAX:-16MiB}"
# Keep the standalone default fail-closed. The Orin wrapper explicitly selects
# "default" because stock JetPack 5 caps the host receive buffer below 16 MiB;
# CycloneDDS still requests GO2_DDS_RCVBUF_MAX and the live sample preflight
# remains the acceptance gate.
GO2_DDS_RCVBUF_MIN="${GO2_DDS_RCVBUF_MIN:-16MiB}"
GO2_DDS_SNDBUF_MAX="${GO2_DDS_SNDBUF_MAX:-16MiB}"
GO2_DDS_FRAGMENT_SIZE="${GO2_DDS_FRAGMENT_SIZE:-65000B}"
GO2_DDS_ALLOW_MULTICAST="${GO2_DDS_ALLOW_MULTICAST:-true}"
export GO2_DDS_RCVBUF_MAX
export GO2_DDS_RCVBUF_MIN
export GO2_DDS_SNDBUF_MAX
export GO2_DDS_FRAGMENT_SIZE
export GO2_DDS_ALLOW_MULTICAST

validate_dds_iface() {
  local iface="$1"
  if [[ ! "${iface}" =~ ^[[:alnum:]_.:-]+$ ]]; then
    echo "CycloneDDS interface contains unsupported characters: ${iface}" >&2
    return 2
  fi
}

build_dds_interfaces_xml() {
  local -a extra_ifaces=()
  local iface
  local normalized_extra_ifaces="${GO2_DDS_EXTRA_IFACES//,/ }"
  local seen_ifaces=" ${GO2_NET_IFACE} "

  validate_dds_iface "${GO2_NET_IFACE}" || return
  printf '  <NetworkInterface name="%s" priority="default" multicast="%s" />\n' \
    "${GO2_NET_IFACE}" "${GO2_DDS_ALLOW_MULTICAST}"

  read -r -a extra_ifaces <<<"${normalized_extra_ifaces}"
  for iface in "${extra_ifaces[@]}"; do
    validate_dds_iface "${iface}" || return
    if [[ "${seen_ifaces}" == *" ${iface} "* ]]; then
      continue
    fi
    printf '  <NetworkInterface name="%s" priority="default" multicast="%s" />\n' \
      "${iface}" "${GO2_DDS_ALLOW_MULTICAST}"
    seen_ifaces+="${iface} "
  done
}

GO2_DDS_INTERFACES_XML="$(build_dds_interfaces_xml)" || return

has_distinct_extra_iface() {
  local -a extra_ifaces=()
  local iface
  local normalized_extra_ifaces="${GO2_DDS_EXTRA_IFACES//,/ }"

  read -r -a extra_ifaces <<<"${normalized_extra_ifaces}"
  for iface in "${extra_ifaces[@]}"; do
    if [[ -n "${iface}" && "${iface}" != "${GO2_NET_IFACE}" ]]; then
      return 0
    fi
  done
  return 1
}

validate_dds_ipv4_address() {
  local address="$1"
  local -a octets=()
  local octet

  IFS='.' read -r -a octets <<<"${address}"
  if [[ "${#octets[@]}" -ne 4 ]]; then
    echo "CycloneDDS primary address must be an IPv4 address: ${address}" >&2
    return 2
  fi
  for octet in "${octets[@]}"; do
    if [[ ! "${octet}" =~ ^[0-9]+$ ]] || (( 10#${octet} > 255 )); then
      echo "CycloneDDS primary address must be an IPv4 address: ${address}" >&2
      return 2
    fi
  done
}

detect_go2_primary_address() {
  if [[ -n "${GO2_DDS_PRIMARY_ADDRESS}" ]]; then
    printf '%s\n' "${GO2_DDS_PRIMARY_ADDRESS}"
    return
  fi

  local address
  address="$(ip -o -4 address show dev "${GO2_NET_IFACE}" 2>/dev/null | awk '
    {
      split($4, parts, "/")
      print parts[1]
      exit
    }
  ')"
  if [[ -n "${address}" ]]; then
    printf '%s\n' "${address}"
    return
  fi

  echo "Could not detect an IPv4 address for GO2_NET_IFACE=${GO2_NET_IFACE}; set GO2_DDS_PRIMARY_ADDRESS explicitly." >&2
  return 2
}

validate_dds_topic_pattern() {
  local pattern="$1"
  if [[ ! "${pattern}" =~ ^[[:alnum:]_./?*:-]+$ ]]; then
    echo "CycloneDDS robot topic pattern contains unsupported characters: ${pattern}" >&2
    return 2
  fi
}

build_dds_robot_partitioning_xml() {
  local primary_address="$1"
  local -a topic_patterns=()
  local pattern
  local normalized_patterns="${GO2_DDS_ROBOT_TOPIC_PATTERNS//,/ }"
  local seen_patterns=" "

  [[ -n "${primary_address}" ]] || return 0

  printf '<Partitioning><NetworkPartitions>\n'
  printf '  <NetworkPartition Name="go2_primary" Address="%s" />\n' \
    "${primary_address}"
  printf '</NetworkPartitions><PartitionMappings>\n'

  read -r -a topic_patterns <<<"${normalized_patterns}"
  for pattern in "${topic_patterns[@]}"; do
    [[ -n "${pattern}" ]] || continue
    validate_dds_topic_pattern "${pattern}" || return
    if [[ "${seen_patterns}" == *" ${pattern} "* ]]; then
      continue
    fi
    printf '  <PartitionMapping DCPSPartitionTopic="*.%s" NetworkPartition="go2_primary" />\n' \
      "${pattern}"
    seen_patterns+="${pattern} "
  done
  printf '</PartitionMappings></Partitioning>\n'
}

GO2_DDS_PARTITIONING_XML=""
if has_distinct_extra_iface; then
  GO2_DDS_PRIMARY_ADDRESS="$(detect_go2_primary_address)" || return
  validate_dds_ipv4_address "${GO2_DDS_PRIMARY_ADDRESS}" || return
  export GO2_DDS_PRIMARY_ADDRESS
  GO2_DDS_PARTITIONING_XML="$(
    build_dds_robot_partitioning_xml "${GO2_DDS_PRIMARY_ADDRESS}"
  )" || return
fi

build_dds_peers_xml() {
  local -a peers=()
  local peer
  local normalized_peers="${GO2_DDS_PEERS//,/ }"
  local seen_peers=" "

  read -r -a peers <<<"${normalized_peers}"
  for peer in "${peers[@]}"; do
    validate_dds_iface "${peer}" || return
    if [[ "${seen_peers}" == *" ${peer} "* ]]; then
      continue
    fi
    printf '  <Peer Address="%s" />\n' "${peer}"
    seen_peers+="${peer} "
  done
}

case "${GO2_DDS_PARTICIPANT_INDEX}" in
  auto|default|none)
    ;;
  *)
    if [[ ! "${GO2_DDS_PARTICIPANT_INDEX}" =~ ^[0-9]+$ ]]; then
      echo "GO2_DDS_PARTICIPANT_INDEX must be auto, default, none, or a nonnegative integer." >&2
      return 2
    fi
    ;;
esac

if [[ ! "${GO2_DDS_MAX_AUTO_PARTICIPANT_INDEX}" =~ ^[0-9]+$ ]] || \
   (( 10#${GO2_DDS_MAX_AUTO_PARTICIPANT_INDEX} > 120 )); then
  echo "GO2_DDS_MAX_AUTO_PARTICIPANT_INDEX must be an integer from 0 through 120." >&2
  return 2
fi

GO2_DDS_PEERS_XML="$(build_dds_peers_xml)" || return

export CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces>
${GO2_DDS_INTERFACES_XML}
</Interfaces><AllowMulticast>${GO2_DDS_ALLOW_MULTICAST}</AllowMulticast><MaxMessageSize>65500B</MaxMessageSize><FragmentSize>${GO2_DDS_FRAGMENT_SIZE}</FragmentSize></General>
<Discovery><ParticipantIndex>${GO2_DDS_PARTICIPANT_INDEX}</ParticipantIndex><MaxAutoParticipantIndex>${GO2_DDS_MAX_AUTO_PARTICIPANT_INDEX}</MaxAutoParticipantIndex><Peers>
${GO2_DDS_PEERS_XML}
</Peers></Discovery>
<Internal><SocketReceiveBufferSize min=\"${GO2_DDS_RCVBUF_MIN}\" max=\"${GO2_DDS_RCVBUF_MAX}\" /><SocketSendBufferSize min=\"default\" max=\"${GO2_DDS_SNDBUF_MAX}\" /></Internal>
${GO2_DDS_PARTITIONING_XML}
</Domain></CycloneDDS>"
