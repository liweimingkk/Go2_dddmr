/*
 * BSD 3-Clause License
 *
 * Copyright (c) 2024, DDDMobileRobot
 * All rights reserved.
 */

#ifndef DDDMR_PG_MAP_SERVER__MAP_ARTIFACT_IDENTITY_H_
#define DDDMR_PG_MAP_SERVER__MAP_ARTIFACT_IDENTITY_H_

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace dddmr_pg_map_server
{

struct MapArtifact
{
  // Stable path relative to the pose-graph root.  The absolute host path is
  // deliberately excluded so moving an unchanged map does not change its ID.
  std::string logical_path;
  std::string filesystem_path;
};

struct MapArtifactIdentityResult
{
  bool valid{false};
  std::string sha256;
  std::string reason;
  std::size_t artifact_count{0U};
  std::uint64_t total_bytes{0U};
};

// Hash a framed, lexicographically ordered manifest of logical path + exact
// file bytes.  Framing includes fixed-width path/content lengths, preventing
// ambiguous concatenations.  File-system locations and timestamps are not
// part of the identity.
MapArtifactIdentityResult computeMapArtifactIdentity(
  const std::vector<MapArtifact> & artifacts);

// Exposed for deterministic pure tests and for documenting the digest
// primitive used by the manifest above.
std::string computeSha256Hex(const std::string & bytes);

}  // namespace dddmr_pg_map_server

#endif  // DDDMR_PG_MAP_SERVER__MAP_ARTIFACT_IDENTITY_H_
