#include <gtest/gtest.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include "map_artifact_identity.h"

namespace dddmr_pg_map_server
{
namespace
{

class MapArtifactIdentityTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    const auto nonce = std::chrono::steady_clock::now().time_since_epoch().count();
    root_ = std::filesystem::temp_directory_path() /
      ("dddmr_map_identity_" + std::to_string(nonce));
    std::filesystem::create_directories(root_ / "copy");
  }

  void TearDown() override
  {
    std::error_code error;
    std::filesystem::remove_all(root_, error);
  }

  std::string write(
    const std::filesystem::path & relative_path, const std::string & bytes)
  {
    const auto path = root_ / relative_path;
    std::filesystem::create_directories(path.parent_path());
    std::ofstream output(path, std::ios::binary);
    output.write(bytes.data(), static_cast<std::streamsize>(bytes.size()));
    output.close();
    return path.string();
  }

  std::filesystem::path root_;
};

TEST(MapArtifactSha256, MatchesPublishedSha256TestVectors)
{
  EXPECT_EQ(
    computeSha256Hex(""),
    "e3b0c44298fc1c149afbf4c8996fb924"
    "27ae41e4649b934ca495991b7852b855");
  EXPECT_EQ(
    computeSha256Hex("abc"),
    "ba7816bf8f01cfea414140de5dae2223"
    "b00361a396177a9cb410ff61f20015ad");
}

TEST_F(MapArtifactIdentityTest, IsIndependentOfInputOrderAndHostDirectory)
{
  const auto poses = write("poses.pcd", "poses\n123");
  const auto ground = write(
    "pcd/0_ground.pcd", std::string{"ground\0bytes", 12U});
  const auto poses_copy = write("copy/poses.pcd", "poses\n123");
  const auto ground_copy = write(
    "copy/0_ground.pcd", std::string{"ground\0bytes", 12U});

  const auto first = computeMapArtifactIdentity({
    {"poses.pcd", poses},
    {"pcd/0_ground.pcd", ground}});
  const auto reordered_and_moved = computeMapArtifactIdentity({
    {"pcd/0_ground.pcd", ground_copy},
    {"poses.pcd", poses_copy}});

  ASSERT_TRUE(first.valid) << first.reason;
  ASSERT_TRUE(reordered_and_moved.valid) << reordered_and_moved.reason;
  EXPECT_EQ(first.sha256, reordered_and_moved.sha256);
  EXPECT_EQ(first.artifact_count, 2U);
  EXPECT_EQ(first.total_bytes, 21U);
}

TEST_F(MapArtifactIdentityTest, ContentAndLogicalPathAreIdentityInputs)
{
  const auto artifact = write("poses.pcd", "same-size-A");
  const auto original = computeMapArtifactIdentity({{"poses.pcd", artifact}});
  ASSERT_TRUE(original.valid) << original.reason;

  write("poses.pcd", "same-size-B");
  const auto changed_content = computeMapArtifactIdentity({{"poses.pcd", artifact}});
  const auto changed_logical_path = computeMapArtifactIdentity({
    {"renamed-poses.pcd", artifact}});

  ASSERT_TRUE(changed_content.valid) << changed_content.reason;
  ASSERT_TRUE(changed_logical_path.valid) << changed_logical_path.reason;
  EXPECT_NE(original.sha256, changed_content.sha256);
  EXPECT_NE(changed_content.sha256, changed_logical_path.sha256);
}

TEST_F(MapArtifactIdentityTest, RejectsMissingEmptyAndDuplicateManifests)
{
  EXPECT_FALSE(computeMapArtifactIdentity({}).valid);
  EXPECT_FALSE(computeMapArtifactIdentity({{"poses.pcd", ""}}).valid);
  EXPECT_FALSE(computeMapArtifactIdentity({
    {"poses.pcd", write("first", "one")},
    {"poses.pcd", write("second", "two")}}).valid);
  EXPECT_FALSE(computeMapArtifactIdentity({
    {"poses.pcd", (root_ / "missing").string()}}).valid);
}

}  // namespace
}  // namespace dddmr_pg_map_server
