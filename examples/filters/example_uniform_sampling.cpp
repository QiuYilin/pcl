/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2019-, Open Perception, Inc.
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#include <pcl/common/generate.h>
#include <pcl/common/random.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <chrono>

using namespace pcl::common;

int
main()
{
  constexpr int SEED = 1234;
  CloudGenerator<pcl::PointXYZ, UniformGenerator<float>> generator;
  UniformGenerator<float>::Parameters x_params(0, 1, SEED + 1);
  generator.setParametersForX(x_params);
  UniformGenerator<float>::Parameters y_params(0, 1, SEED + 2);
  generator.setParametersForY(y_params);
  UniformGenerator<float>::Parameters z_params(0, 1, SEED + 3);
  generator.setParametersForZ(z_params);
  pcl::PointCloud<pcl::PointXYZ>::Ptr xyz(new pcl::PointCloud<pcl::PointXYZ>);
  generator.fill(100, 100, *xyz);

  // The generated cloud points are distributed in the unit cube. By using 0.1 sized
  // voxels for sampling, we divide each side of the cube into 1 / 0.1 = 10 cells, in
  // total 10^3 = 1000 cells. Since we generated a large amount of points, we can be
  // sure that each cell has at least one point. As a result, we expect 1000 points in
  // the output cloud and the rest in removed indices.

    pcl::UniformSampling<pcl::PointXYZ> us(true); // extract removed indices
  us.setInputCloud(xyz);
  us.setRadiusSearch(0.1);
  pcl::PointCloud<pcl::PointXYZ> output;

  //cloud

// Measure the execution time
auto start = std::chrono::high_resolution_clock::now();

us.filter(output);

auto end = std::chrono::high_resolution_clock::now();
auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
std::cout << "Execution time: " << duration.count() << " milliseconds" << std::endl;

// Visualization
pcl::visualization::PCLVisualizer viewer("Point Cloud Viewer");
viewer.setBackgroundColor(0.0, 0.0, 0.0);
viewer.addPointCloud<pcl::PointXYZ>(output.makeShared(), "cloud");
viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");
viewer.addCoordinateSystem(1.0);
viewer.initCameraParameters();
while (!viewer.wasStopped()) {
    viewer.spinOnce();
}

  //indices
  pcl::Indices indices;
  us.filter(indices);//由于索引是用map存储的，所以不同次的输出是无序的


//   for (std::size_t i = 0; i < 10; ++i)
//   {
//     // Compare original points with sampled indices against sampled points
//     // EXPECT_NEAR ((*xyz)[indices[i]].x, output[i].x, 1e-4);
//     // EXPECT_NEAR ((*xyz)[indices[i]].y, output[i].y, 1e-4);
//     // EXPECT_NEAR ((*xyz)[indices[i]].z, output[i].z, 1e-4);
//     std::cout << "Original: " << (*xyz)[indices[i]].x << " " << (*xyz)[indices[i]].y << " " << (*xyz)[indices[i]].z << std::endl;
//     std::cout << "Sampled: " << output[i].x << " " << output[i].y << " " << output[i].z << std::endl;
//   }

  auto removed_indices = us.getRemovedIndices();
//   ASSERT_EQ(output.size(), 1000);
//   EXPECT_EQ(int(removed_indices->size()), int(xyz->size() - 1000));
  std::set<int> removed_indices_set(removed_indices->begin(), removed_indices->end());
//   ASSERT_EQ(removed_indices_set.size(), removed_indices->size());

  // Negative
  us.setNegative (true);
  us.filter(output);
  removed_indices = us.getRemovedIndices ();
//   EXPECT_EQ (int (removed_indices->size ()), 1000);
//   EXPECT_EQ (int (output.size ()), int (xyz->size() - 1000));

  //Organized
  us.setKeepOrganized (true);
  us.setNegative (false);
  us.filter(output);
  removed_indices = us.getRemovedIndices ();
//   EXPECT_EQ (int (removed_indices->size ()), int(xyz->size() - 1000));
//   for (std::size_t i = 0; i < removed_indices->size (); ++i)
//   {
//     EXPECT_TRUE (std::isnan (output.at ((*removed_indices)[i]).x));
//     EXPECT_TRUE (std::isnan (output.at ((*removed_indices)[i]).y));
//     EXPECT_TRUE (std::isnan (output.at ((*removed_indices)[i]).z));
//   }

//   EXPECT_EQ (output.width, xyz->width);
//   EXPECT_EQ (output.height, xyz->height);

  // Check input cloud with nan values
  us.setInputCloud (output.makeShared ());
  us.setRadiusSearch(2);
  us.filter (output);
  removed_indices = us.getRemovedIndices ();
//   ASSERT_EQ(output.size(), 1);
std::cout << "removed indices" << removed_indices->size() << std::endl;

//   EXPECT_EQ (int (removed_indices->size ()), 1000-1);
}
