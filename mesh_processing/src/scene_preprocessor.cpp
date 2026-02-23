#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>

#include <rcl_interfaces/msg/set_parameters_result.hpp>

using PointT = pcl::PointXYZ;
using CloudT = pcl::PointCloud<PointT>;

class ScenePreprocessor : public rclcpp::Node
{
public:
    ScenePreprocessor() : Node("scene_preprocessor")
    {
        // -----------------------------
        // Declare parameters
        // -----------------------------
        declare_parameter("voxel_size", 0.005);
        declare_parameter("z_min", 0.2);
        declare_parameter("z_max", 1.0);
        declare_parameter("x_min", -0.5);
        declare_parameter("x_max", 0.5);
        declare_parameter("y_min", -0.5);
        declare_parameter("y_max", 0.5);
        declare_parameter("plane_thresh", 0.015);

        // 🔥 New parameters
        declare_parameter("use_sor", true);
        declare_parameter("sor_mean_k", 30);
        declare_parameter("sor_stddev", 1.0);

        declare_parameter("use_normals", true);
        declare_parameter("normal_radius", 0.02);

        declare_parameter("plane_axis_x", 0.0);
        declare_parameter("plane_axis_y", 0.0);
        declare_parameter("plane_axis_z", 1.0);
        declare_parameter("plane_eps_angle", 0.15); // radians (~8.5°)

        update_params();

        param_callback_handle_ =
            add_on_set_parameters_callback(
                std::bind(&ScenePreprocessor::parametersCallback, this, std::placeholders::_1));

        // ✅ Correct QoS for RealSense
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
    // Parameters
    double voxel_size_, z_min_, z_max_, x_min_, x_max_, y_min_, y_max_;
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
    rcl_interfaces::msg::SetParametersResult
    parametersCallback(const std::vector<rclcpp::Parameter> &params)
    {
        for (const auto &p : params)
        {
            if (p.get_name() == "voxel_size")
                voxel_size_ = p.as_double();
            else if (p.get_name() == "z_min")
                z_min_ = p.as_double();
            else if (p.get_name() == "z_max")
                z_max_ = p.as_double();
            else if (p.get_name() == "x_min")
                x_min_ = p.as_double();
            else if (p.get_name() == "x_max")
                x_max_ = p.as_double();
            else if (p.get_name() == "y_min")
                y_min_ = p.as_double();
            else if (p.get_name() == "y_max")
                y_max_ = p.as_double();
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
        z_min_ = get_parameter("z_min").as_double();
        z_max_ = get_parameter("z_max").as_double();
        x_min_ = get_parameter("x_min").as_double();
        x_max_ = get_parameter("x_max").as_double();
        y_min_ = get_parameter("y_min").as_double();
        y_max_ = get_parameter("y_max").as_double();
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

        // 1️⃣ Voxel
        pcl::VoxelGrid<PointT> voxel;
        voxel.setInputCloud(cloud);
        voxel.setLeafSize(voxel_size_, voxel_size_, voxel_size_);
        voxel.filter(*cloud);

        if (cloud->empty())
            return;

        // 2️⃣ ROI filtering
        pcl::PassThrough<PointT> pass;
        pass.setInputCloud(cloud);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(z_min_, z_max_);
        pass.filter(*cloud);

        pass.setInputCloud(cloud);
        pass.setFilterFieldName("x");
        pass.setFilterLimits(x_min_, x_max_);
        pass.filter(*cloud);

        pass.setInputCloud(cloud);
        pass.setFilterFieldName("y");
        pass.setFilterLimits(y_min_, y_max_);
        pass.filter(*cloud);

        if (cloud->empty())
            return;

        // 3️⃣ SOR
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

        // 4️⃣ Perpendicular Plane RANSAC
        pcl::SACSegmentation<PointT> seg;
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr coeffs(new pcl::ModelCoefficients);

        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setMaxIterations(100);
        seg.setDistanceThreshold(plane_thresh_);

        seg.setAxis(Eigen::Vector3f(axis_x_, axis_y_, axis_z_));
        seg.setEpsAngle(plane_eps_angle_);

        seg.setInputCloud(cloud);
        seg.segment(*inliers, *coeffs);

        if (!inliers->indices.empty())
        {
            pcl::ExtractIndices<PointT> extract;
            extract.setInputCloud(cloud);
            extract.setIndices(inliers);
            extract.setNegative(true);
            extract.filter(*cloud);
        }

        if (cloud->empty())
            return;

        // 5️⃣ Normals (optional)
        if (use_normals_)
        {
            pcl::NormalEstimation<PointT, pcl::Normal> ne;
            pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
            pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);

            ne.setInputCloud(cloud);
            ne.setSearchMethod(tree);
            ne.setRadiusSearch(normal_radius_);
            ne.compute(*normals);
        }

        // Publish
        sensor_msgs::msg::PointCloud2 output;
        pcl::toROSMsg(*cloud, output);
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