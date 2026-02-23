#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <tf2_ros/static_transform_broadcaster.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <pcl/features/normal_3d_omp.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <random>

using Cloud = pcl::PointCloud<pcl::PointXYZ>;
using CloudN = pcl::PointCloud<pcl::PointNormal>;

class ModelManager : public rclcpp::Node
{
public:
    ModelManager() : Node("model_manager")
    {
        this->declare_parameter<std::string>("stl_path", "mesh/puzzel.stl");
        this->declare_parameter<int>("sample_points", 120000);

        this->declare_parameter<bool>("auto_scale", true);
        this->declare_parameter<double>("scale_threshold", 10.0);

        this->declare_parameter<bool>("center_model", true);

        this->declare_parameter<double>("voxel_percentage", 0.005);  // 0.5%
        this->declare_parameter<double>("voxel_size_override", 0.0); // 0 = disabled

        this->declare_parameter<double>("normal_radius_multiplier", 3.0);

        rclcpp::QoS qos(1);
        qos.transient_local();
        qos.reliable();

        cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("model_cloud", qos);
        path_pub_ = this->create_publisher<std_msgs::msg::String>("model_path", qos);

        load_service_ = this->create_service<std_srvs::srv::Trigger>(
            "load_model",
            std::bind(&ModelManager::loadCallback, this,
                      std::placeholders::_1, std::placeholders::_2));

        tf_broadcaster_ =
            std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

        broadcastStaticTF();
        loadModelFromParameter();

        RCLCPP_INFO(this->get_logger(), "Model Manager Ready.");
    }

private:
    // ============================================================
    // Static TF
    // ============================================================
    void broadcastStaticTF()
    {
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = this->now();
        t.header.frame_id = "world";
        t.child_frame_id = "model_frame";
        t.transform.rotation.w = 1.0;
        tf_broadcaster_->sendTransform(t);
    }

    // ============================================================
    // AREA-WEIGHTED UNIFORM SURFACE SAMPLING
    // ============================================================
    Cloud::Ptr sampleMeshUniform(const pcl::PolygonMesh &mesh, int n_samples)
    {
        Cloud::Ptr cloud(new Cloud);

        pcl::PointCloud<pcl::PointXYZ> vertices;
        pcl::fromPCLPointCloud2(mesh.cloud, vertices);

        std::vector<double> cumulative_area;
        cumulative_area.reserve(mesh.polygons.size());

        double total_area = 0.0;

        // Compute triangle areas
        for (const auto &poly : mesh.polygons)
        {
            const auto &p0 = vertices[poly.vertices[0]];
            const auto &p1 = vertices[poly.vertices[1]];
            const auto &p2 = vertices[poly.vertices[2]];

            Eigen::Vector3f v0(p0.x, p0.y, p0.z);
            Eigen::Vector3f v1(p1.x, p1.y, p1.z);
            Eigen::Vector3f v2(p2.x, p2.y, p2.z);

            double area = 0.5 * ((v1 - v0).cross(v2 - v0)).norm();
            total_area += area;
            cumulative_area.push_back(total_area);
        }

        std::mt19937 gen(std::random_device{}());
        std::uniform_real_distribution<double> dist_area(0.0, total_area);
        std::uniform_real_distribution<float> dist_unit(0.0f, 1.0f);

        // Sample proportional to triangle area
        for (int i = 0; i < n_samples; ++i)
        {
            double r = dist_area(gen);

            auto tri_it = std::lower_bound(
                cumulative_area.begin(),
                cumulative_area.end(),
                r);

            int tri_index = std::distance(cumulative_area.begin(), tri_it);
            const auto &poly = mesh.polygons[tri_index];

            const auto &p0 = vertices[poly.vertices[0]];
            const auto &p1 = vertices[poly.vertices[1]];
            const auto &p2 = vertices[poly.vertices[2]];

            float r1 = std::sqrt(dist_unit(gen));
            float r2 = dist_unit(gen);

            pcl::PointXYZ p;
            p.x = (1 - r1) * p0.x + r1 * (1 - r2) * p1.x + r1 * r2 * p2.x;
            p.y = (1 - r1) * p0.y + r1 * (1 - r2) * p1.y + r1 * r2 * p2.y;
            p.z = (1 - r1) * p0.z + r1 * (1 - r2) * p1.z + r1 * r2 * p2.z;

            cloud->push_back(p);
        }

        return cloud;
    }

    // ============================================================
    void loadModelFromParameter()
    {
        std::string relative =
            this->get_parameter("stl_path").as_string();

        std::string pkg_path =
            ament_index_cpp::get_package_share_directory("mesh_processing");

        std::string full_path = pkg_path + "/" + relative;

        process(full_path);
    }

    // ============================================================
    void loadCallback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        loadModelFromParameter();
        response->success = true;
        response->message = "Model reloaded";
    }

    // ============================================================
    void process(const std::string &filename)
    {
        pcl::PolygonMesh mesh;
        if (pcl::io::loadPolygonFileSTL(filename, mesh) == 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to load STL.");
            return;
        }

        int samples = this->get_parameter("sample_points").as_int();
        Cloud::Ptr model = sampleMeshUniform(mesh, samples);

        // ---------------- Scale Detection ----------------
        Eigen::Vector4f min_pt, max_pt;
        pcl::getMinMax3D(*model, min_pt, max_pt);

        float max_range = std::max({std::abs(min_pt.x()), std::abs(max_pt.x()),
                                    std::abs(min_pt.y()), std::abs(max_pt.y()),
                                    std::abs(min_pt.z()), std::abs(max_pt.z())});

        bool auto_scale = this->get_parameter("auto_scale").as_bool();
        double scale_threshold = this->get_parameter("scale_threshold").as_double();

        if (auto_scale && max_range > scale_threshold)
        {
            RCLCPP_WARN(this->get_logger(),
                        "Large model detected (%.2f). Scaling by 0.001",
                        max_range);

            for (auto &p : model->points)
            {
                p.x *= 0.001f;
                p.y *= 0.001f;
                p.z *= 0.001f;
            }
        }

        pcl::getMinMax3D(*model, min_pt, max_pt);

        float model_size = std::max({max_pt.x() - min_pt.x(),
                                     max_pt.y() - min_pt.y(),
                                     max_pt.z() - min_pt.z()});

        double voxel_override = this->get_parameter("voxel_size_override").as_double();
        double voxel_percentage = this->get_parameter("voxel_percentage").as_double();

        float voxel_size;

        if (voxel_override > 0.0)
        {
            voxel_size = voxel_override;
        }
        else
        {
            voxel_size = model_size * voxel_percentage;
        }

        RCLCPP_INFO(this->get_logger(),
                    "Model size: %.4f m | Voxel: %.5f m",
                    model_size, voxel_size);

        // ---------------- Center ----------------
        bool center_model = this->get_parameter("center_model").as_bool();

        if (center_model)
        {
            Eigen::Vector4f centroid;
            pcl::compute3DCentroid(*model, centroid);

            Eigen::Affine3f tf = Eigen::Affine3f::Identity();
            tf.translation() = -centroid.head<3>();
            pcl::transformPointCloud(*model, *model, tf);
        }

        // ---------------- Downsample ----------------
        pcl::VoxelGrid<pcl::PointXYZ> voxel;
        voxel.setInputCloud(model);
        voxel.setLeafSize(voxel_size, voxel_size, voxel_size);

        Cloud::Ptr filtered(new Cloud);
        voxel.filter(*filtered);

        // ---------------- Normals ----------------
        pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
        ne.setNumberOfThreads(4);
        ne.setInputCloud(filtered);

        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
            new pcl::search::KdTree<pcl::PointXYZ>());

        ne.setSearchMethod(tree);
        double normal_mult = this->get_parameter("normal_radius_multiplier").as_double();
        ne.setRadiusSearch(voxel_size * normal_mult);

        pcl::PointCloud<pcl::Normal>::Ptr normals(
            new pcl::PointCloud<pcl::Normal>());

        ne.compute(*normals);

        CloudN::Ptr output(new CloudN);
        pcl::concatenateFields(*filtered, *normals, *output);

        // ---------------- Publish ----------------
        sensor_msgs::msg::PointCloud2 msg;
        pcl::toROSMsg(*output, msg);
        msg.header.frame_id = "model_frame";
        msg.header.stamp = this->now();
        cloud_pub_->publish(msg);

        std_msgs::msg::String path_msg;
        path_msg.data = filename;
        path_pub_->publish(path_msg);

        RCLCPP_INFO(this->get_logger(),
                    "Published %ld points.", output->size());
    }

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr path_pub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr load_service_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ModelManager>());
    rclcpp::shutdown();
    return 0;
}