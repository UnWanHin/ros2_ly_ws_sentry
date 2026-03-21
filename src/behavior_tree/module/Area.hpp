// AUTO-COMMENT: file overview
// This file belongs to the ROS2 sentry workspace codebase.
// Keep behavior and interface changes synchronized with related modules.

//re-build 不用改
#pragma once

#include <vector>
#include <limits>
#include <cstdint>
#include "BasicTypes.hpp"

using namespace LangYa;


namespace BehaviorTree {

namespace Area {
    
    // 定义 concept，限制 T 是基本算术类型（int、double、float 等）
    template<typename T>
    concept Arithmetic = std::is_arithmetic_v<T>;

    template<Arithmetic T>
    struct Point {
        T x;
        T y;
    };

    template<Arithmetic T>
    class Location {
    public:
        Location() = default;
        explicit Location(const Point<T>& pointRed, const Point<T>& pointBlue)
            : pointRed_(pointRed), pointBlue_(pointBlue) {}

        Point<T> operator()(const UnitTeam team) const { 
            return team == UnitTeam::Red ? pointRed_ : pointBlue_;
        }

        // 判断给定点 (x, y) 是否位于该点的区域内
        bool near(const T x, const T y, const T distance, UnitTeam team) const {
            const auto& point = (team == UnitTeam::Red) ? pointRed_ : pointBlue_;
            T dx = x - point.x;
            T dy = y - point.y;
            T distanceSquared =  dx * dx + dy * dy;
            return distanceSquared < distance * distance;
        }

    private:
        Point<T> pointRed_;
        Point<T> pointBlue_;
    };

    template<Arithmetic T>
    class Area {
    public:
        Area() = default;
        explicit Area(std::vector<Point<T>> points)
            : boundaryPoints_(std::move(points)) {
            if (boundaryPoints_.empty()) {
                throw std::invalid_argument("Points vector cannot be empty.");
            }
        }
        ~Area() = default;

        void addBoundaryPoint(T x, T y) {
            boundaryPoints_.emplace_back(x, y);
        }

        bool isPointInside(T px, T py) const {
            if (boundaryPoints_.empty()) return false;

            bool inside = false;
            for (size_t i = 0, j = boundaryPoints_.size() - 1; i < boundaryPoints_.size(); j = i++) {
                T xi = boundaryPoints_[i].x, yi = boundaryPoints_[i].y;
                T xj = boundaryPoints_[j].x, yj = boundaryPoints_[j].y;

                // 避免除以零的情况
                if (yi == yj) continue;

                if ((yi > py) != (yj > py)) {
                    // 判断点是否在边的左侧
                    if (px < (xj - xi) * (py - yi) / static_cast<T>(yj - yi) + xi) {
                        inside = !inside;
                    }
                }
            }
            return inside;
        }

    private:
        std::vector<Point<T>> boundaryPoints_;
    };

    static const std::vector<Point<std::uint16_t>> CastleRedPoints = {
        { 583, 749 },
        { 624, 823 },
        { 707, 823 },
        { 748, 753 },
        { 715, 679 },
        { 620, 675 }
    }; // 红方堡垒

    static const std::vector<Point<std::uint16_t>> CastleBluePoints = {
        { 2041, 749 },
        { 2086, 826 },
        { 2171, 826 },
        { 2211, 753 },
        { 2171, 672 },
        { 2082, 680 }
    }; // 蓝方堡垒

    static const std::vector<Point<std::uint16_t>> CentralHighlandRedPoints = {
        { 1031, 527 },
        { 1027, 947 },
        { 1257, 1263 },
        { 1605, 1242 },
        { 1597, 1016 },
        { 1346, 794 }

    }; // 中部高地-红方

    static const std::vector<Point<std::uint16_t>> CentralHighlandBluePoints = {
        { 1726, 963 },
        { 1756, 563 },
        { 1544, 268 },
        { 1181, 268 },
        { 1193, 498 },
        { 1439, 705 }
    }; // 中部高地-蓝方

    static const std::vector<Point<std::uint16_t>> roadLandRedPoints = {
        { 522, 207 },
        { 1201, 207 },
        { 1201, 21 },
        { 522, 21 }
    }; // 公路区-红方

    static const std::vector<Point<std::uint16_t>> roadLandBluePoints = {
        { 2280, 1311 },
        { 1589, 1311 },
        { 1589, 1477 },
        { 2280, 1477}
    }; // 公路区-蓝方

    static const std::vector<Point<std::uint16_t>> baseRedPoints = {
        { 17, 947 },
        { 494, 947 },
        { 494, 527 },
        { 17, 527 }
    }; // 基地-红方

    static const std::vector<Point<std::uint16_t>> baseBluePoints = {
        { 2764, 555 },
        { 2300, 555 },
        { 2300, 980 },
        { 2764, 980}
    }; // 基地-蓝方

    static const std::vector<Point<std::uint16_t>> flyLandRedPoints = {
        { 1261, 118 },
        { 1783, 118 },
        { 1783, 17 },
        { 1261, 17}
    }; // 飞坡区-红方

    static const std::vector<Point<std::uint16_t>> flyLandBluePoints = {
        { 1532, 1384 },
        { 1019, 1384 },
        { 1019, 1489 },
        { 1532, 1489}
    }; // 飞坡区-蓝方

    // 特殊区域
    static const Area<std::uint16_t> CastleRed{CastleRedPoints};
    static const Area<std::uint16_t> CastleBlue{CastleBluePoints};
    static const Area<std::uint16_t> CentralHighLandRed{CentralHighlandRedPoints};
    static const Area<std::uint16_t> CentralHighLandBlue{CentralHighlandBluePoints};
    static const Area<std::uint16_t> RoadLandRed{roadLandRedPoints};
    static const Area<std::uint16_t> RoadLandBlue{roadLandBluePoints};
    static const Area<std::uint16_t> BaseRed{baseRedPoints};
    static const Area<std::uint16_t> BaseBlue{baseBluePoints};
    static const Area<std::uint16_t> FlyLandRed{flyLandRedPoints};
    static const Area<std::uint16_t> FlyLandBlue{flyLandBluePoints};

    // 特殊点 {Red, Blue}
    static const Location<std::uint16_t> Home{ {393, 810}, {2408, 683} };
    static const Location<std::uint16_t> Base{ {401, 691}, {2400, 811} };
    static const Location<std::uint16_t> Recovery{ {183, 245}, {2619, 1249} };
    static const Location<std::uint16_t> BuffShoot{ {924, 1388}, {1872, 115} };
    static const Location<std::uint16_t> LeftHighLand{ {406, 1332}, {2392, 187} };
    static const Location<std::uint16_t> CastleLeft{ {657, 952}, {2136, 542} };
    static const Location<std::uint16_t> Castle{ {666, 749}, {2132, 749} };
    static const Location<std::uint16_t> CastleRight1{ {505, 497}, {2293, 1014} };
    static const Location<std::uint16_t> CastleRight2{ {831, 509}, {1971, 985} };
    static const Location<std::uint16_t> FlyRoad{ {868, 80} , {1929, 1402}};
    static const Location<std::uint16_t> OutpostArea{ {1138, 431}, {1635, 1079} };
    static const Location<std::uint16_t> MidShoot{ {1075, 898}, {1702, 609} };
    static const Location<std::uint16_t> LeftShoot{ {1165, 1063}, {1644, 476} };
    static const Location<std::uint16_t> OutpostShoot{ {1393, 1047}, {1434, 497} };
    static const Location<std::uint16_t> BuffAround1{ {1372, 952}, {1459, 530} };
    static const Location<std::uint16_t> BuffAround2{ {1203, 770}, {1591, 753} };
    static const Location<std::uint16_t> RightShoot{ {1240, 559}, {1549, 923} };
    static const Location<std::uint16_t> HoleRoad{ {1074, 1249}, {1677, 204} };
    // 联盟赛推荐走 /ly/navi/goal=OccupyArea，由导航侧解释为“占点区域”。
    // 这里保留一个兼容坐标，占位到中场附近，避免旧链路在 UseXY=true 时无定义。
    static const Location<std::uint16_t> OccupyArea{ {1075, 898}, {1702, 609} };



} // namespace Area
} // namespace BehaviorTree

//似乎是場地的類似信息收集，一些特別的點位?
