#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

// Filters
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h> // 🚀 Replaces PassThrough
#include <pcl/filters/statistical_outlier_removal.h>

// Segmentation
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

// Features & Utilities
#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>
#include <pcl/common/io.h> // 🚀 Required for concatenateFields

#include <rcl_interfaces/msg/set_parameters_result.hpp>

// Define Point Types
using PointT = pcl::PointXYZ;
using CloudT = pcl::PointCloud<PointT>;
using PointNormalT = pcl::PointNormal;
using CloudNormalT = pcl::PointCloud<PointNormalT>;

class ScenePreprocessor : public rclcpp::Node
{
public:
    ScenePreprocessor() : Node("scene_preprocessor")
    {
        // -----------------------------
        // Declare Parameters (Matching Python Script)
        // -----------------------------

        // 1. ROI Bounding Box Parameters (Gazebo Frame)
        declare_parameter("x_min", 0.24);
        declare_parameter("x_max", 0.26);
        declare_parameter("y_min", -0.10);
        declare_parameter("y_max", 0.10);
        declare_parameter("z_min", -0.10);
        declare_parameter("z_max", 0.10);

        // 2. Voxel Size
        declare_parameter("voxel_size", 0.001);

        // 3. Statistical Outlier Removal
        declare_parameter("use_sor", true);
        declare_parameter("sor_mean_k", 30);
        declare_parameter("sor_stddev", 1.0);

        // 4. Table Plane Segmentation
        declare_parameter("plane_thresh", 0.001);
        declare_parameter("plane_axis_x", 1.0); // Table normal aligns with X-axis in Gazebo optical frame
        declare_parameter("plane_axis_y", 0.0);
        declare_parameter("plane_axis_z", 0.0);
        declare_parameter("plane_eps_angle", 0.15); // radians (~8.5 degrees tolerance)

        // 5. Normal Estimation
        declare_parameter("use_normals", true);
        declare_parameter("normal_radius", 0.02);

        // Initialize local variables from parameter server
        update_params();

        // Bind parameter callback for live tuning via CLI
        param_callback_handle_ =
            add_on_set_parameters_callback(
                std::bind(&ScenePreprocessor::parametersCallback, this, std::placeholders::_1));

        // ✅ Correct QoS for RealSense / Gazebo
        auto qos = rclcpp::QoS(rclcpp::KeepLast(5))
                       .best_effort()
                       .durability_volatile();

        subscriber_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            "/camera/depth/color/points",
            qos,
            std::bind(&ScenePreprocessor::cloud_callback, this, std::placeholders::_1));

        publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
            "/processed_scene",
            rclcpp::QoS(10));

        RCLCPP_INFO(get_logger(), "🚀 Fully Optimized Scene Preprocessor Ready.");
    }

private:
    // Local Parameter Storage
    double voxel_size_, x_min_, x_max_, y_min_, y_max_, z_min_, z_max_;
    double plane_thresh_;
    bool use_sor_, use_normals_;
    int sor_mean_k_;
    double sor_stddev_, normal_radius_;
    double axis_x_, axis_y_, axis_z_, plane_eps_angle_;

    OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscriber_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;

    // -----------------------------
    // Parameter callback (LIVE tuning)
    // -----------------------------
    rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter> &params)
    {
        for (const auto &p : params)
        {
            if (p.get_name() == "voxel_size")
                voxel_size_ = p.as_double();
            else if (p.get_name() == "x_min")
                x_min_ = p.as_double();
            else if (p.get_name() == "x_max")
                x_max_ = p.as_double();
            else if (p.get_name() == "y_min")
                y_min_ = p.as_double();
            else if (p.get_name() == "y_max")
                y_max_ = p.as_double();
            else if (p.get_name() == "z_min")
                z_min_ = p.as_double();
            else if (p.get_name() == "z_max")
                z_max_ = p.as_double();
            else if (p.get_name() == "plane_thresh")
                plane_thresh_ = p.as_double();
            else if (p.get_name() == "use_sor")
                use_sor_ = p.as_bool();
            else if (p.get_name() == "sor_mean_k")
                sor_mean_k_ = p.as_int();
            else if (p.get_name() == "sor_stddev")
                sor_stddev_ = p.as_double();
            else if (p.get_name() == "use_normals")
                use_normals_ = p.as_bool();
            else if (p.get_name() == "normal_radius")
                normal_radius_ = p.as_double();
            else if (p.get_name() == "plane_axis_x")
                axis_x_ = p.as_double();
            else if (p.get_name() == "plane_axis_y")
                axis_y_ = p.as_double();
            else if (p.get_name() == "plane_axis_z")
                axis_z_ = p.as_double();
            else if (p.get_name() == "plane_eps_angle")
                plane_eps_angle_ = p.as_double();
        }
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        return result;
    }

    void update_params()
    {
        voxel_size_ = get_parameter("voxel_size").as_double();
        x_min_ = get_parameter("x_min").as_double();
        x_max_ = get_parameter("x_max").as_double();
        y_min_ = get_parameter("y_min").as_double();
        y_max_ = get_parameter("y_max").as_double();
        z_min_ = get_parameter("z_min").as_double();
        z_max_ = get_parameter("z_max").as_double();
        plane_thresh_ = get_parameter("plane_thresh").as_double();
        use_sor_ = get_parameter("use_sor").as_bool();
        sor_mean_k_ = get_parameter("sor_mean_k").as_int();
        sor_stddev_ = get_parameter("sor_stddev").as_double();
        use_normals_ = get_parameter("use_normals").as_bool();
        normal_radius_ = get_parameter("normal_radius").as_double();
        axis_x_ = get_parameter("plane_axis_x").as_double();
        axis_y_ = get_parameter("plane_axis_y").as_double();
        axis_z_ = get_parameter("plane_axis_z").as_double();
        plane_eps_angle_ = get_parameter("plane_eps_angle").as_double();
    }

    // -----------------------------
    // Cloud processing pipeline
    // -----------------------------
    void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        CloudT::Ptr cloud(new CloudT);
        pcl::fromROSMsg(*msg, *cloud);

        if (cloud->empty())
            return;

        // 1️⃣ CropBox (MUST BE FIRST TO PREVENT MEMORY OVERFLOW)
        pcl::CropBox<PointT> crop;
        crop.setInputCloud(cloud);
        crop.setMin(Eigen::Vector4f(x_min_, y_min_, z_min_, 1.0f));
        crop.setMax(Eigen::Vector4f(x_max_, y_max_, z_max_, 1.0f));
        crop.filter(*cloud);

        if (cloud->empty())
            return;

        // 2️⃣ Voxel Downsampling (Safe now that the cloud is cropped)
        pcl::VoxelGrid<PointT> voxel;
        voxel.setInputCloud(cloud);
        voxel.setLeafSize(voxel_size_, voxel_size_, voxel_size_);
        voxel.filter(*cloud);

        if (cloud->empty())
            return;

        // 3️⃣ Statistical Outlier Removal (SOR)
        if (use_sor_)
        {
            pcl::StatisticalOutlierRemoval<PointT> sor;
            sor.setInputCloud(cloud);
            sor.setMeanK(sor_mean_k_);
            sor.setStddevMulThresh(sor_stddev_);
            sor.filter(*cloud);
        }

        if (cloud->empty())
            return;

        // 4️⃣ RANSAC Plane Segmentation (Delete the Table)
        pcl::SACSegmentation<PointT> seg;
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr coeffs(new pcl::ModelCoefficients);

        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setMaxIterations(1000); // Matched to Python
        seg.setDistanceThreshold(plane_thresh_);
        seg.setAxis(Eigen::Vector3f(axis_x_, axis_y_, axis_z_));
        seg.setEpsAngle(plane_eps_angle_);

        seg.setInputCloud(cloud);
        seg.segment(*inliers, *coeffs);

        // If a table is found, remove it
        if (!inliers->indices.empty())
        {
            pcl::ExtractIndices<PointT> extract;
            extract.setInputCloud(cloud);
            extract.setIndices(inliers);
            extract.setNegative(true); // Keep everything EXCEPT the table
            extract.filter(*cloud);
        }

        if (cloud->empty())
            return;

        // 5️⃣ Normals & Publishing
        sensor_msgs::msg::PointCloud2 output;

        if (use_normals_)
        {
            pcl::NormalEstimation<PointT, pcl::Normal> ne;
            pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
            pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);

            ne.setInputCloud(cloud);
            ne.setSearchMethod(tree);
            ne.setRadiusSearch(normal_radius_);

            // Orient all normal vectors toward the camera lens (0,0,0)
            ne.setViewPoint(0.0, 0.0, 0.0);
            ne.compute(*normals);

            // 🚀 CRITICAL FIX: Merge the XYZ coordinates and the Normal vectors together
            CloudNormalT::Ptr cloud_with_normals(new CloudNormalT);
            pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);

            pcl::toROSMsg(*cloud_with_normals, output);
        }
        else
        {
            pcl::toROSMsg(*cloud, output);
        }

        // Attach the original timestamp and frame ID
        output.header = msg->header;
        publisher_->publish(output);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ScenePreprocessor>());
    rclcpp::shutdown();
    return 0;
}