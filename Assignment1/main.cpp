#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>

constexpr double MY_PI = 3.1415926;

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0], 
                 0, 1, 0, -eye_pos[1], 
                 0, 0, 1, -eye_pos[2], 
                 0, 0, 0, 1;

    view = translate * view;

    return view;
}

Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the model matrix for rotating the triangle around the Z axis.
    // Then return it.

    float rad = rotation_angle / 180.0 * MY_PI;

    model << std::cos(rad), -std::sin(rad), 0, 0,
             std::sin(rad),  std::cos(rad), 0, 0,
             0,              0,             1, 0,
             0,              0,             0, 1;

    return model;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar)
{
    // Students will implement this function

    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the projection matrix for the given parameters.
    // Then return it.

    float t = std::tan(eye_fov / 2.0 / 180.0 * MY_PI) * std::abs(zNear);
    float r = t * aspect_ratio;
    float l = -r;
    float b = -t;

    Eigen::Matrix4f persp2ortho;
    persp2ortho << zNear, 0, 0, 0,
                   0, zNear, 0, 0,
                   0, 0, zNear + zFar, -zNear * zFar,
                   0, 0, 1, 0;

    Eigen::Matrix4f ortho;
    ortho << 2 / (r - l), 0, 0, -(r + l) / 2,
             0, 2 / (t - b), 0, -(t + b) / 2,
             0, 0, 2 / (zNear - zFar), -(zNear + zFar) / 2,
             0, 0, 0, 1;

    projection = ortho * persp2ortho;

    return projection;
}

Eigen::Matrix4f get_rotation(Eigen::Vector3f axis, float angle)
{
    Eigen::Matrix4f rotation = Eigen::Matrix4f::Identity();

    Eigen::Vector3f n = axis.normalized();
    float rad = angle / 180.0 * MY_PI;

    Eigen::Matrix3f I = Eigen::Matrix3f::Identity();
    Eigen::Matrix3f nnT = n * n.transpose();
    Eigen::Matrix3f N;
    N << 0, -n[2], n[1],
         n[2], 0, -n[0],
        -n[1], n[0], 0;
    
    Eigen::Matrix3f R = std::cos(rad) * I + (1 - std::cos(rad)) * nnT + std::sin(rad) * N;

    rotation.block<3,3>(0,0) = R;
    rotation(3,3) = 1.0f;

    return rotation;
}

int main(int argc, const char** argv)
{
    float angle = 0;
    bool command_line = false;
    std::string filename = "output.png";

    if (argc >= 3) {
        command_line = true;
        angle = std::stof(argv[2]); // -r by default
        if (argc == 4) {
            filename = std::string(argv[3]);
        }
    }

    rst::rasterizer r(700, 700);

    Eigen::Vector3f eye_pos = {0, 0, 5};

    std::vector<Eigen::Vector3f> pos{{2, 0, -2}, {0, 2, -2}, {-2, 0, -2}};

    std::vector<Eigen::Vector3i> ind{{0, 1, 2}};

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);

    int key = 0;
    int frame_count = 0;

    Eigen::Vector3f axis = {0, 0, 1};

    if (command_line) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        // r.set_model(get_model_matrix(angle));
        r.set_model(get_rotation(axis, angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);

        cv::imwrite(filename, image);

        return 0;
    }

    while (key != 27) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        // r.set_model(get_model_matrix(angle));
        r.set_model(get_rotation(axis, angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::imshow("image", image);
        key = cv::waitKey(10);

        std::cout << "frame count: " << frame_count++ << '\n';

        if (key == 'a') {
            angle += 10;
        }
        else if (key == 'd') {
            angle -= 10;
        }
        // 按 'x' 切换到 X 轴
        else if (key == 'x') {
            axis = {1, 0, 0};
            std::cout << "Switched to X-Axis rotation.\n";
        }
        // 按 'y' 切换到 Y 轴
        else if (key == 'y') {
            axis = {0, 1, 0};
            std::cout << "Switched to Y-Axis rotation.\n";
        }
        // 按 'z' 切换到 Z 轴
        else if (key == 'z') {
            axis = {0, 0, 1};
            std::cout << "Switched to Z-Axis rotation.\n";
        }
        // 按 'c' (Custom) 自定义输入
        else if (key == 'c') {
            std::cout << "\n=== Enter Custom Axis (x y z) ===\n";
            std::cout << "Input: ";
            float x, y, z;
            std::cin >> x >> y >> z; 
            axis = {x, y, z};
            std::cout << "Axis updated to: (" << x << ", " << y << ", " << z << ")\n";
            std::cout << "Click the image window to resume control.\n";
        }
    }

    return 0;
}
