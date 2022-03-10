//
// Created by ascend on 2021/2/26.
//

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/progressive_morphological_filter.h>

#include <pcl/filters/morphological_filter.h>
#include <pcl/pcl_base.h>
using namespace std;

#define max_window_size 0.05
#define slope 0.7f
#define max_distance 0.1f
#define initial_distance 0.01f
#define cell_size 0.01f
#define base 2.0f
#define exponential true

pcl::PointCloud<pcl::PointXYZ>::Ptr pmf (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in)
{
    // Compute the series of window sizes and height thresholds
    std::vector<float> height_thresholds;
    std::vector<float> window_sizes;
    std::vector<int> ground_indices;
    int iteration = 0;
    float window_size = 0.0f;
    float height_threshold = 0.0f;

    while (window_size < max_window_size)
    {
        // Determine the initial window size.
        if (exponential)
            window_size = cell_size * (2.0f * std::pow (base, iteration) + 1.0f);
        else
            window_size = cell_size * (2.0f * (iteration+1) * base + 1.0f);
        cout << "window_size  " << window_size  << endl;
        // Calculate the height threshold to be used in the next iteration.
        if (iteration == 0)
            height_threshold = initial_distance;
        else
            height_threshold = slope * (window_size - window_sizes[iteration-1]) * cell_size + initial_distance;
        cout << "height_threshold  " << height_threshold  << endl;


        // Enforce max distance on height threshold
        if (height_threshold > max_distance)
            height_threshold = max_distance;

        window_sizes.push_back (window_size);
        height_thresholds.push_back (height_threshold);

        iteration++;
    }

    // Ground indices are initially limited to those points in the input cloud we
    // wish to process
    for (int i=0;i<cloud_in->points.size();i++){
        ground_indices.push_back(i);
    }

    // Progressively filter ground returns using morphological open
    for (size_t i = 0; i < window_sizes.size (); ++i)
    {
        cout<< "Iteration " << i << "height threshold = " << height_thresholds[i] << " window size = " <<
            window_sizes[i] << endl;

        // Limit filtering to those points currently considered ground returns
        typename pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::copyPointCloud<pcl::PointXYZ> (*cloud_in, ground_indices, *cloud);

        // Create new cloud to hold the filtered results. Apply the morphological
        // opening operation at the current window size.
        typename pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::applyMorphologicalOperator<pcl::PointXYZ> (cloud, window_sizes[i], pcl::MORPH_OPEN, *cloud_f);

        // Find indices of the points whose difference between the source and
        // filtered point clouds is less than the current height threshold.
        std::vector<int> pt_indices;
        //cout << "ground.size() = " << ground.size() << endl;
        for (size_t p_idx = 0; p_idx < ground_indices.size (); ++p_idx)
        {
            float diff = cloud->points[p_idx].z - cloud_f->points[p_idx].z;
            //cout << "diff " << diff << endl;
            if (diff < height_thresholds[i])
                pt_indices.push_back (ground_indices[p_idx]);
        }

        // Ground is now limited to pt_indices
        ground_indices.swap (pt_indices);
        cout << "ground now has " << ground_indices.size () << " points" << endl;
    }
    typename pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);
    // Extract cloud_in with ground indices
    pcl::copyPointCloud<pcl::PointXYZ> (*cloud_in, ground_indices, *cloud_out);
    return cloud_out;
}


int main (int argc, char** argv)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointIndicesPtr ground (new pcl::PointIndices);

    // Fill in the cloud data
    pcl::PCDReader reader;
    // Replace the path below with the path where you saved your file
    reader.read<pcl::PointXYZ> (argv[1], *cloud);

    std::cerr << "Cloud before filtering: " << std::endl;
    std::cerr << *cloud << std::endl;

    // Run progressive morphological filter
    cloud_filtered = pmf(cloud);

    std::cerr << "Ground cloud after filtering: " << std::endl;
    std::cerr << *cloud_filtered << std::endl;

    pcl::PCDWriter writer;
    writer.write<pcl::PointXYZ> ("ground.pcd", *cloud_filtered, false);

    // Extract non-ground returns
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud (cloud);
    extract.setIndices (ground);
    //extract.filter (*cloud_filtered);
    extract.setNegative (true);
    extract.filter (*cloud_filtered);

    std::cerr << "Object cloud after filtering: " << std::endl;
    std::cerr << *cloud_filtered << std::endl;

    writer.write<pcl::PointXYZ> ("object.pcd", *cloud_filtered, false);

    return (0);
}
