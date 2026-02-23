#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>

#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/sample_consensus_prerejective.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

using PointT = pcl::PointXYZ;
using PointNormalT = pcl::PointNormal;
using FeatureT = pcl::FPFHSignature33;

using CloudT = pcl::PointCloud<PointT>;
using CloudNormalT = pcl::PointCloud<PointNormalT>;
using FeatureCloudT = pcl::PointCloud<FeatureT>;

class PoseEstimator : public rclcpp::Node
{
public:
    PoseEstimator() : Node("pose_estimator"), global_alignment_done_(false)
    {
        // -----------------------------
        // Declare parameters
        // -----------------------------
        this->declare_parameter("normal_radius", 0.02);
        this->declare_parameter("feature_radius", 0.05);
        this->declare_parameter("icp_max_iterations", 50);
        this->declare_parameter("icp_max_correspondence_dist", 0.05);
        this->declare_parameter("debug_mode", false);

        // --- NEW: Pose Locking Parameters to filter noise ---
        this->declare_parameter("lock_translation_threshold", 0.003); // 3 mm
        this->declare_parameter("lock_rotation_threshold", 0.035);    // ~2 degrees (in radians)

        // --- NEW: RANSAC Parameters from previous tuning ---
        this->declare_parameter("ransac_max_corr_dist", 0.05);
        this->declare_parameter("ransac_inlier_fraction", 0.25);
        this->declare_parameter("ransac_max_iterations", 50000);

        // -----------------------------
        // Subscriptions & Publishers
        // -----------------------------
        // Subscribing to the ModelManager's output (STL cloud)
        model_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/model_cloud", rclcpp::QoS(1).transient_local(),
            std::bind(&PoseEstimator::model_callback, this, std::placeholders::_1));

        // Subscribing to the ScenePreprocessor's output (RealSense cloud)
        scene_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/processed_scene", 10,
            std::bind(&PoseEstimator::scene_callback, this, std::placeholders::_1));

        // Publish the aligned model for visualization in RViz
        aligned_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/aligned_model", 10);
        debug_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/debug_model_in_tf", 10);

        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        model_cloud_.reset(new CloudNormalT);
        model_features_.reset(new FeatureCloudT);

        current_transform_ = Eigen::Matrix4f::Identity();
        locked_transform_ = Eigen::Matrix4f::Identity();

        RCLCPP_INFO(this->get_logger(), "Pose Estimator Node Started. Waiting for data...");
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr model_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr scene_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr aligned_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr debug_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    CloudNormalT::Ptr model_cloud_;
    FeatureCloudT::Ptr model_features_;
    bool has_model_ = false;
    bool global_alignment_done_;

    Eigen::Matrix4f current_transform_;
    Eigen::Matrix4f locked_transform_; // The stable pose used to filter out noise

    // ============================================================
    // Model Callback (Runs once when STL is published)
    // ============================================================
    void model_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        if (has_model_)
            return; // Only process the model once

        pcl::fromROSMsg(*msg, *model_cloud_);

        // ModelManager already computed normals, so we just compute FPFH features
        double feature_radius = this->get_parameter("feature_radius").as_double();
        compute_features(model_cloud_, model_features_, feature_radius);

        has_model_ = true;
        RCLCPP_INFO(this->get_logger(), "Model received and features computed. Ready for scene data.");
    }

    // ============================================================
    // Scene Callback (Runs continuously for live RealSense data)
    // ============================================================
    void scene_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        if (!has_model_)
            return; // Wait until we have the STL model

        // 1. Convert ROS message to PCL PointXYZ
        CloudT::Ptr raw_scene(new CloudT);
        pcl::fromROSMsg(*msg, *raw_scene);

        if (raw_scene->empty())
            return;

        // 2. ScenePreprocessor only outputs PointXYZ. We MUST compute normals for Point-to-Plane ICP
        CloudNormalT::Ptr scene_cloud(new CloudNormalT);
        double normal_radius = this->get_parameter("normal_radius").as_double();
        compute_normals(raw_scene, scene_cloud, normal_radius);

        // 3. Registration Pipeline
        Eigen::Matrix4f transformation_guess = Eigen::Matrix4f::Identity();
        bool debug = this->get_parameter("debug_mode").as_bool();

        if (!global_alignment_done_)
        {
            // === STAGE 1: GLOBAL REGISTRATION ===
            FeatureCloudT::Ptr scene_features(new FeatureCloudT);
            double feature_radius = this->get_parameter("feature_radius").as_double();
            compute_features(scene_cloud, scene_features, feature_radius);

            if (debug)
                RCLCPP_INFO(this->get_logger(), "[DEBUG] Running Global Registration (FPFH + RANSAC)...");

            pcl::SampleConsensusPrerejective<PointNormalT, PointNormalT, FeatureT> align;
            align.setInputSource(model_cloud_);
            align.setSourceFeatures(model_features_);
            align.setInputTarget(scene_cloud);
            align.setTargetFeatures(scene_features);

            // Map tuning parameters
            align.setMaximumIterations(this->get_parameter("ransac_max_iterations").as_int());
            align.setNumberOfSamples(3);
            align.setCorrespondenceRandomness(5);
            align.setSimilarityThreshold(0.9f);
            align.setMaxCorrespondenceDistance(this->get_parameter("ransac_max_corr_dist").as_double());
            align.setInlierFraction(this->get_parameter("ransac_inlier_fraction").as_double());

            CloudNormalT::Ptr aligned_global(new CloudNormalT);
            align.align(*aligned_global);

            if (align.hasConverged())
            {
                transformation_guess = align.getFinalTransformation();
                global_alignment_done_ = true;

                // Initialize the locked transform to this first successful guess
                locked_transform_ = transformation_guess;

                if (debug)
                    RCLCPP_INFO(this->get_logger(), "[DEBUG] Global Registration Successful! Fitness score: %f", align.getFitnessScore());
            }
            else
            {
                if (debug)
                    RCLCPP_WARN(this->get_logger(), "[DEBUG] Global Registration Failed. Will retry next frame.");
                return;
            }
        }
        else
        {
            // === TEMPORAL TRACKING ===
            if (debug)
                RCLCPP_INFO(this->get_logger(), "[DEBUG] Skipping Global Registration. Using Temporal Tracking.");
            // Use the perfectly stable locked position as our starting point
            transformation_guess = locked_transform_;
        }

        // === STAGE 2: LOCAL REGISTRATION (Point-to-Plane ICP) ===
        if (debug)
            RCLCPP_INFO(this->get_logger(), "[DEBUG] Running Point-to-Plane ICP...");
        pcl::IterativeClosestPointWithNormals<PointNormalT, PointNormalT> icp;
        icp.setInputSource(model_cloud_);
        icp.setInputTarget(scene_cloud);

        icp.setMaxCorrespondenceDistance(this->get_parameter("icp_max_correspondence_dist").as_double());
        icp.setMaximumIterations(this->get_parameter("icp_max_iterations").as_int());
        icp.setTransformationEpsilon(1e-8);
        icp.setEuclideanFitnessEpsilon(1e-6);

        CloudNormalT::Ptr icp_aligned_cloud(new CloudNormalT);
        icp.align(*icp_aligned_cloud, transformation_guess);

        if (icp.hasConverged())
        {
            Eigen::Matrix4f new_transform = icp.getFinalTransformation();
            if (debug)
                RCLCPP_INFO(this->get_logger(), "[DEBUG] ICP Converged! Fitness score: %f", icp.getFitnessScore());

            // --- NEW: POSE LOCKING & NOISE FILTERING MATH ---

            // 1. Translation Error (Euclidean Distance)
            Eigen::Vector3f t_new = new_transform.block<3, 1>(0, 3);
            Eigen::Vector3f t_locked = locked_transform_.block<3, 1>(0, 3);
            float delta_t = (t_new - t_locked).norm();

            // 2. Rotation Error (Angle between Quaternions)
            Eigen::Quaternionf q_new(new_transform.block<3, 3>(0, 0));
            Eigen::Quaternionf q_locked(locked_transform_.block<3, 3>(0, 0));
            float dot = std::abs(q_new.dot(q_locked));
            dot = std::min(1.0f, dot); // Clamp to prevent NaN in acos due to floating point math
            float delta_r = 2.0f * std::acos(dot);

            // 3. Evaluate Thresholds
            float trans_thresh = this->get_parameter("lock_translation_threshold").as_double();
            float rot_thresh = this->get_parameter("lock_rotation_threshold").as_double();

            if (delta_t > trans_thresh || delta_r > rot_thresh)
            {
                // The object genuinely moved! Update the lock.
                locked_transform_ = new_transform;
                if (debug)
                    RCLCPP_INFO(this->get_logger(), "[DEBUG] Object MOVED! Updating pose. (dt: %.4fm, dR: %.4frad)", delta_t, delta_r);
            }
            else
            {
                // The shift is smaller than the threshold. It's just camera noise. Keep the lock.
                if (debug)
                    RCLCPP_INFO(this->get_logger(), "[DEBUG] Pose LOCKED. Filtering out noise. (dt: %.4fm, dR: %.4frad)", delta_t, delta_r);
            }

            // Always use the stable locked transform for the rest of the system
            current_transform_ = locked_transform_;

            // Transform the raw model cloud using the STABLE transform, avoiding the jittery ICP cloud output
            CloudNormalT::Ptr final_stable_cloud(new CloudNormalT);
            pcl::transformPointCloud(*model_cloud_, *final_stable_cloud, current_transform_);

            // Publish the stable aligned point cloud to RViz
            sensor_msgs::msg::PointCloud2 aligned_msg;
            pcl::toROSMsg(*final_stable_cloud, aligned_msg);
            aligned_msg.header.frame_id = msg->header.frame_id;
            aligned_msg.header.stamp = this->now();
            aligned_pub_->publish(aligned_msg);

            // Broadcast the calculated, rock-solid 6D pose as a TF Frame
            broadcast_tf(current_transform_, msg->header.frame_id, "estimated_object_frame", msg->header.stamp);

            // Publish the original STL model in the new TF frame for debugging
            if (debug)
            {
                sensor_msgs::msg::PointCloud2 debug_msg;
                pcl::toROSMsg(*model_cloud_, debug_msg);
                debug_msg.header.frame_id = "estimated_object_frame";
                debug_msg.header.stamp = msg->header.stamp;
                debug_pub_->publish(debug_msg);
            }
        }
        else
        {
            if (debug)
                RCLCPP_WARN(this->get_logger(), "[DEBUG] ICP lost tracking. Resetting to Global Registration.");
            global_alignment_done_ = false; // Reset to find it again next frame
        }
    }

    // ============================================================
    // Helper: Compute Normals
    // ============================================================
    void compute_normals(const CloudT::Ptr &input, CloudNormalT::Ptr &output, double radius)
    {
        pcl::NormalEstimationOMP<PointT, pcl::Normal> ne;
        pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
        pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());

        ne.setInputCloud(input);
        ne.setSearchMethod(tree);
        ne.setRadiusSearch(radius);
        ne.compute(*normals);

        // Concatenate XYZ and Normals into PointNormal
        pcl::concatenateFields(*input, *normals, *output);
    }

    // ============================================================
    // Helper: Compute FPFH Features
    // ============================================================
    void compute_features(const CloudNormalT::Ptr &input, FeatureCloudT::Ptr &features, double radius)
    {
        pcl::FPFHEstimationOMP<PointNormalT, PointNormalT, FeatureT> fpfh;
        pcl::search::KdTree<PointNormalT>::Ptr tree(new pcl::search::KdTree<PointNormalT>());

        fpfh.setInputCloud(input);
        fpfh.setInputNormals(input);
        fpfh.setSearchMethod(tree);
        fpfh.setRadiusSearch(radius);
        fpfh.compute(*features);
    }

    // ============================================================
    // Helper: Broadcast Transformation Matrix to TF2
    // ============================================================
    void broadcast_tf(const Eigen::Matrix4f &transform, const std::string &parent_frame, const std::string &child_frame, rclcpp::Time stamp)
    {
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = stamp;
        t.header.frame_id = parent_frame;
        t.child_frame_id = child_frame;

        // Extract translation
        t.transform.translation.x = transform(0, 3);
        t.transform.translation.y = transform(1, 3);
        t.transform.translation.z = transform(2, 3);

        // Extract rotation (Matrix to Quaternion)
        Eigen::Matrix3f rot_matrix = transform.block<3, 3>(0, 0);
        Eigen::Quaternionf q(rot_matrix);
        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();
        t.transform.rotation.w = q.w();

        tf_broadcaster_->sendTransform(t);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PoseEstimator>());
    rclcpp::shutdown();
    return 0;
}