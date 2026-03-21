// AUTO-COMMENT: file overview
// This file belongs to the ROS2 sentry workspace codebase.
// Keep behavior and interface changes synchronized with related modules.

#include "controller/MuzzleSolver.hpp"
namespace CONTROLLER
{
        MuzzleSolver::MuzzleSolver(Eigen::Vector3d gun_muzzle_offset)
    {
        this->gun_muzzle_offset = gun_muzzle_offset;
        this->time_delay = 0.10;
    }

    MuzzleSolver::~MuzzleSolver()
    {
        // Destructor implementation
    }

    float MuzzleSolver::bulletModel(float dis, float angle)
    {
        float k_wind = 0.001, k_gravity = 9.8;
        float t = (exp(k_wind * dis) - 1) / (k_wind * this->bullet_speed * cos(angle));
        float z = this->bullet_speed * sin(angle) * t - 0.5 * k_gravity * t * t;
        return z;
        
    }

    BoardInformations MuzzleSolver::solveMuzzle(PREDICTOR::OutpostInformation &outpost_information)
    {
        BoardInformations board_infos;
        // Implement the logic to solve the gun muzzle position and orientation
        // based on the vehicle information and IMU flag.

        outpost_information.center_position[0] += gun_muzzle_offset[0];
        outpost_information.center_position[1] += gun_muzzle_offset[1];
        outpost_information.center_position[2] += gun_muzzle_offset[2];

        // 根据center_position求出子弹飞行时间
        float distance_horizontal = sqrt(outpost_information.center_position[0] * outpost_information.center_position[0] 
            + outpost_information.center_position[1] * outpost_information.center_position[1]);
        float pitch = atan2(outpost_information.center_position[2], distance_horizontal);
        float time_fly = abs(distance_horizontal / this->bullet_speed * cos(pitch));

        time_fly += this->time_delay;
        std::cout << "time_fly: " << time_fly << std::endl;

        outpost_information.center_position[0] += outpost_information.center_velocity[0] * time_fly;
        outpost_information.center_position[1] += outpost_information.center_velocity[1] * time_fly;
        outpost_information.center_position[2] += outpost_information.center_velocity[2] * time_fly;

        outpost_information.outpost_theta += outpost_information.outpost_omega * time_fly;

        // 计算三块装甲板的位置 ,x_a = x_c - r *sin(theta),y_a = y_c - r * cos(theta)
        double radius = outpost_information.outpost_radius;
        double z = outpost_information.center_position[2];
        Eigen::Vector3d board_position[3];
        for (int i = 0; i < 3; i++)
        {
            if(outpost_information.outpost_omega>0){
                board_position[i][0] = outpost_information.center_position[0] - radius * sin(outpost_information.outpost_theta + i * 2 * M_PI / 3);
                board_position[i][1] = outpost_information.center_position[1] - radius * cos(outpost_information.outpost_theta + i * 2 * M_PI / 3);
                board_position[i][2] = z;
            }else{
                board_position[i][0] = outpost_information.center_position[0] + radius * sin(outpost_information.outpost_theta + i * 2 * M_PI / 3);
                board_position[i][1] = outpost_information.center_position[1] - radius * cos(outpost_information.outpost_theta + i * 2 * M_PI / 3);
                board_position[i][2] = z;
            }

        }

        // 添加子弹速度检查
        if(this->bullet_speed <= 0) {
            COUT("Invalid bullet speed", RED);
            return BoardInformations();
        }
        
        for (int i = 0; i < 2; i++)
        {
            BoardInformation board_info;
            board_info.aim_yaw = -atan2(board_position[i][0], board_position[i][1]);
            float z_temp = board_position[i][2];
            float distance_horizontal = sqrt(board_position[i][0] * board_position[i][0] + board_position[i][1] * board_position[i][1]);
            board_info.aim_distance = distance_horizontal;
            float dz, a, z_actual;
            for (int j = 0; j < 20; j++) {
                a = atan2(z_temp, distance_horizontal);
                z_actual = this->bulletModel(distance_horizontal, a);
                dz = board_position[i][2] - z_actual;
                z_temp = z_temp + dz;
                if (abs(dz) < 0.001)
                    break;
            }
            board_info.aim_pitch = atan2(z_temp, distance_horizontal);

            cv::Point2f aim_center_derection = cv::Point2f(outpost_information.center_position[0], outpost_information.center_position[1]);
            cv::Point2f center_armor_derection = cv::Point2f(board_position[i][0] - outpost_information.center_position[0], board_position[i][1] - outpost_information.center_position[1]);
            if (center_armor_derection.y>0) continue;
            // 计算夹角的cos值
            //double cosA = -(aim_center_derection.x * center_armor_derection.x + aim_center_derection.y * center_armor_derection.y) / (sqrt(aim_center_derection.x * aim_center_derection.x + aim_center_derection.y * aim_center_derection.y) * sqrt(center_armor_derection.x * center_armor_derection.x + center_armor_derection.y * center_armor_derection.y));
            double angle = abs(atan2(center_armor_derection.x,abs(center_armor_derection.y)));
            board_info.face_cos = angle;

            board_infos.push_back(board_info);
        }

        return board_infos;
    }

} // namespace CONTROLLER