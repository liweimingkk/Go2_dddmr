/*
 * BSD 3-Clause License
 *
 * Copyright (c) 2024, DDDMobileRobot
 * All rights reserved.
 */

#include "map_artifact_identity.h"

#include <algorithm>
#include <array>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <limits>
#include <set>
#include <sstream>
#include <utility>

namespace dddmr_pg_map_server
{
namespace
{

constexpr std::array<std::uint32_t, 64> kRoundConstants{{
  0x428a2f98U, 0x71374491U, 0xb5c0fbcfU, 0xe9b5dba5U,
  0x3956c25bU, 0x59f111f1U, 0x923f82a4U, 0xab1c5ed5U,
  0xd807aa98U, 0x12835b01U, 0x243185beU, 0x550c7dc3U,
  0x72be5d74U, 0x80deb1feU, 0x9bdc06a7U, 0xc19bf174U,
  0xe49b69c1U, 0xefbe4786U, 0x0fc19dc6U, 0x240ca1ccU,
  0x2de92c6fU, 0x4a7484aaU, 0x5cb0a9dcU, 0x76f988daU,
  0x983e5152U, 0xa831c66dU, 0xb00327c8U, 0xbf597fc7U,
  0xc6e00bf3U, 0xd5a79147U, 0x06ca6351U, 0x14292967U,
  0x27b70a85U, 0x2e1b2138U, 0x4d2c6dfcU, 0x53380d13U,
  0x650a7354U, 0x766a0abbU, 0x81c2c92eU, 0x92722c85U,
  0xa2bfe8a1U, 0xa81a664bU, 0xc24b8b70U, 0xc76c51a3U,
  0xd192e819U, 0xd6990624U, 0xf40e3585U, 0x106aa070U,
  0x19a4c116U, 0x1e376c08U, 0x2748774cU, 0x34b0bcb5U,
  0x391c0cb3U, 0x4ed8aa4aU, 0x5b9cca4fU, 0x682e6ff3U,
  0x748f82eeU, 0x78a5636fU, 0x84c87814U, 0x8cc70208U,
  0x90befffaU, 0xa4506cebU, 0xbef9a3f7U, 0xc67178f2U}};

std::uint32_t rotateRight(const std::uint32_t value, const unsigned int count)
{
  return (value >> count) | (value << (32U - count));
}

class Sha256
{
public:
  void update(const std::uint8_t * data, const std::size_t size)
  {
    total_bytes_ += static_cast<std::uint64_t>(size);
    for (std::size_t index = 0U; index < size; ++index) {
      buffer_[buffer_size_++] = data[index];
      if (buffer_size_ == buffer_.size()) {
        transform(buffer_.data());
        buffer_size_ = 0U;
      }
    }
  }

  void update(const std::string & bytes)
  {
    update(
      reinterpret_cast<const std::uint8_t *>(bytes.data()), bytes.size());
  }

  std::array<std::uint8_t, 32> finish()
  {
    const std::uint64_t message_bits = total_bytes_ * 8U;
    const std::uint8_t marker = 0x80U;
    update(&marker, 1U);

    const std::uint8_t zero = 0U;
    while (buffer_size_ != 56U) {
      update(&zero, 1U);
    }

    std::array<std::uint8_t, 8> length_bytes{};
    for (std::size_t index = 0U; index < length_bytes.size(); ++index) {
      length_bytes[length_bytes.size() - 1U - index] =
        static_cast<std::uint8_t>(message_bits >> (index * 8U));
    }
    update(length_bytes.data(), length_bytes.size());

    std::array<std::uint8_t, 32> digest{};
    for (std::size_t word = 0U; word < state_.size(); ++word) {
      digest[word * 4U] = static_cast<std::uint8_t>(state_[word] >> 24U);
      digest[word * 4U + 1U] = static_cast<std::uint8_t>(state_[word] >> 16U);
      digest[word * 4U + 2U] = static_cast<std::uint8_t>(state_[word] >> 8U);
      digest[word * 4U + 3U] = static_cast<std::uint8_t>(state_[word]);
    }
    return digest;
  }

private:
  void transform(const std::uint8_t * block)
  {
    std::array<std::uint32_t, 64> words{};
    for (std::size_t index = 0U; index < 16U; ++index) {
      words[index] =
        (static_cast<std::uint32_t>(block[index * 4U]) << 24U) |
        (static_cast<std::uint32_t>(block[index * 4U + 1U]) << 16U) |
        (static_cast<std::uint32_t>(block[index * 4U + 2U]) << 8U) |
        static_cast<std::uint32_t>(block[index * 4U + 3U]);
    }
    for (std::size_t index = 16U; index < words.size(); ++index) {
      const std::uint32_t sigma0 =
        rotateRight(words[index - 15U], 7U) ^
        rotateRight(words[index - 15U], 18U) ^
        (words[index - 15U] >> 3U);
      const std::uint32_t sigma1 =
        rotateRight(words[index - 2U], 17U) ^
        rotateRight(words[index - 2U], 19U) ^
        (words[index - 2U] >> 10U);
      words[index] = words[index - 16U] + sigma0 + words[index - 7U] + sigma1;
    }

    std::uint32_t a = state_[0U];
    std::uint32_t b = state_[1U];
    std::uint32_t c = state_[2U];
    std::uint32_t d = state_[3U];
    std::uint32_t e = state_[4U];
    std::uint32_t f = state_[5U];
    std::uint32_t g = state_[6U];
    std::uint32_t h = state_[7U];

    for (std::size_t index = 0U; index < words.size(); ++index) {
      const std::uint32_t choice = (e & f) ^ (~e & g);
      const std::uint32_t majority = (a & b) ^ (a & c) ^ (b & c);
      const std::uint32_t sum0 =
        rotateRight(a, 2U) ^ rotateRight(a, 13U) ^ rotateRight(a, 22U);
      const std::uint32_t sum1 =
        rotateRight(e, 6U) ^ rotateRight(e, 11U) ^ rotateRight(e, 25U);
      const std::uint32_t temporary1 =
        h + sum1 + choice + kRoundConstants[index] + words[index];
      const std::uint32_t temporary2 = sum0 + majority;
      h = g;
      g = f;
      f = e;
      e = d + temporary1;
      d = c;
      c = b;
      b = a;
      a = temporary1 + temporary2;
    }

    state_[0U] += a;
    state_[1U] += b;
    state_[2U] += c;
    state_[3U] += d;
    state_[4U] += e;
    state_[5U] += f;
    state_[6U] += g;
    state_[7U] += h;
  }

  std::array<std::uint32_t, 8> state_{{
    0x6a09e667U, 0xbb67ae85U, 0x3c6ef372U, 0xa54ff53aU,
    0x510e527fU, 0x9b05688cU, 0x1f83d9abU, 0x5be0cd19U}};
  std::array<std::uint8_t, 64> buffer_{};
  std::size_t buffer_size_{0U};
  std::uint64_t total_bytes_{0U};
};

void updateUint64(Sha256 & digest, const std::uint64_t value)
{
  std::array<std::uint8_t, 8> bytes{};
  for (std::size_t index = 0U; index < bytes.size(); ++index) {
    bytes[bytes.size() - 1U - index] =
      static_cast<std::uint8_t>(value >> (index * 8U));
  }
  digest.update(bytes.data(), bytes.size());
}

std::string toHex(const std::array<std::uint8_t, 32> & digest)
{
  std::ostringstream stream;
  stream << std::hex << std::setfill('0');
  for (const auto byte : digest) {
    stream << std::setw(2) << static_cast<unsigned int>(byte);
  }
  return stream.str();
}

}  // namespace

std::string computeSha256Hex(const std::string & bytes)
{
  Sha256 digest;
  digest.update(bytes);
  return toHex(digest.finish());
}

MapArtifactIdentityResult computeMapArtifactIdentity(
  const std::vector<MapArtifact> & artifacts)
{
  MapArtifactIdentityResult result;
  if (artifacts.empty()) {
    result.reason = "map artifact manifest is empty";
    return result;
  }

  auto ordered = artifacts;
  std::sort(
    ordered.begin(), ordered.end(),
    [](const MapArtifact & left, const MapArtifact & right) {
      return left.logical_path < right.logical_path;
    });

  std::set<std::string> logical_paths;
  for (const auto & artifact : ordered) {
    if (artifact.logical_path.empty() || artifact.filesystem_path.empty()) {
      result.reason = "map artifact path must not be empty";
      return result;
    }
    if (artifact.logical_path.find('\0') != std::string::npos) {
      result.reason = "map artifact logical path contains a NUL byte";
      return result;
    }
    if (!logical_paths.insert(artifact.logical_path).second) {
      result.reason = "duplicate map artifact logical path: " + artifact.logical_path;
      return result;
    }
  }

  Sha256 digest;
  digest.update(std::string{"DDDMR_MAP_ARTIFACT_MANIFEST_V1\0", 31U});
  updateUint64(digest, static_cast<std::uint64_t>(ordered.size()));

  std::array<char, 64U * 1024U> buffer{};
  for (const auto & artifact : ordered) {
    std::ifstream input(artifact.filesystem_path, std::ios::binary | std::ios::ate);
    if (!input) {
      result.reason = "cannot open map artifact: " + artifact.filesystem_path;
      return result;
    }
    const auto end_position = input.tellg();
    if (end_position < std::streampos{0}) {
      result.reason = "cannot determine map artifact size: " + artifact.filesystem_path;
      return result;
    }
    const auto file_size = static_cast<std::uint64_t>(end_position);
    if (result.total_bytes >
      std::numeric_limits<std::uint64_t>::max() - file_size)
    {
      result.reason = "map artifact byte count overflows uint64";
      return result;
    }
    input.seekg(0, std::ios::beg);

    updateUint64(
      digest, static_cast<std::uint64_t>(artifact.logical_path.size()));
    digest.update(artifact.logical_path);
    updateUint64(digest, file_size);

    std::uint64_t bytes_read = 0U;
    while (input) {
      input.read(buffer.data(), static_cast<std::streamsize>(buffer.size()));
      const auto count = input.gcount();
      if (count > 0) {
        digest.update(
          reinterpret_cast<const std::uint8_t *>(buffer.data()),
          static_cast<std::size_t>(count));
        bytes_read += static_cast<std::uint64_t>(count);
      }
    }
    if (!input.eof() || bytes_read != file_size) {
      result.reason = "map artifact changed or failed while reading: " +
        artifact.filesystem_path;
      return result;
    }
    result.total_bytes += file_size;
  }

  result.valid = true;
  result.artifact_count = ordered.size();
  result.sha256 = toHex(digest.finish());
  return result;
}

}  // namespace dddmr_pg_map_server
