// AUTO-COMMENT: file overview
// This file belongs to the ROS2 sentry workspace codebase.
// Keep behavior and interface changes synchronized with related modules.

#pragma once
#include <cmath>
#include <iostream>
#include <memory>
#include <stdexcept>
#include "Logger/Logger.hpp"

// 定义三种坐标类型
struct XYZ {
    double x, y, z;
};

struct PYD {
    double pitch, yaw, distance;
};

struct CXYD {
    double cx, cy, distance;//when used as XYV , distance > 1 means visible
    CXYD() : cx(0), cy(0), distance(0) {}
    CXYD(double cx, double cy, double distance) : cx(cx), cy(cy), distance(distance) {}
    CXYD(double cx, double cy) : cx(cx), cy(cy), distance(0) {}
};
struct XYV {
    double x, y;
    bool visible;
    XYV() : x(0), y(0), visible(true) {}
    XYV(double x, double y) : x(x), y(y), visible(true) {}
    XYV(CXYD cxy) : x(cxy.cx), y(cxy.cy), visible(cxy.distance > 1) {}
};

// //前向声明
// namespace solver {
//     struct ImuData;
// }


namespace solver {

// 抽象转换接口，由外部实现
class BaseSolver {
public:
    // pyd <-> xyz
    virtual PYD XYZ2PYD(const XYZ &in) const = 0;
    virtual XYZ PYD2XYZ(const PYD &in) const = 0;

    // cxy <-> pyd
    virtual PYD CXYD2PYD(const CXYD &in) const = 0;
    virtual CXYD PYD2CXYD(const PYD &in) const = 0;

    // pyd + imu -> pyd_imu
    virtual PYD fuseIMU(const PYD &p, const PYD &imu) const = 0;
    // pyd_imu + imu -> pyd（即还原非融合的值）
    virtual PYD separateIMU(const PYD &p_imu, const PYD &imu) const = 0;
};

} // namespace solver

// Location类：内部以 base_pyd_ 为非融合基本数据，所有非融合数据均由此计算得到。  
// 融合数据通过 fuseIMU(base_pyd_, imu) 得到，即 fused_pyd_；进而 xyz_imu 与 cxy_imu 由 fused_pyd_ 转换得出。
namespace location {
class Location {
public:
    // 注册全局Solver实例，所有Location实例均使用同一转换器  
    static void registerSolver(const std::shared_ptr<const solver::BaseSolver>& s) {
        solverInstance_ = s;
        solverRegistered_ = (s != nullptr);
    }
  
    // 非融合代理类：pyd、xyz、cxy
    class PYDProxy {
    public:
        PYDProxy(Location &parent) : parent_(parent) {}
        PYDProxy& operator=(const PYD &value) {
            if(!solverRegistered_) {
                roslog::error("Solver not registered. PYD assignment ignored.");
                return *this;
            }
            parent_.base_pyd_ = value;
            parent_.updateFused();
            return *this;
        }
        operator PYD() const { return parent_.base_pyd_; }
    private:
        Location &parent_;
    };

    class XYZProxy {
    public:
        XYZProxy(Location &parent) : parent_(parent) {}
        XYZProxy& operator=(const XYZ &value) {
            if(!solverRegistered_) {
                roslog::error("Solver not registered. XYZ assignment ignored.");
                return *this;
            }
            // 利用XYZ2PYD转换后更新基本变量
            parent_.base_pyd_ = solverInstance_->XYZ2PYD(value);
            parent_.updateFused();
            return *this;
        }
        operator XYZ() const {
            if(!solverRegistered_) return XYZ{0,0,0};
            return solverInstance_->PYD2XYZ(parent_.base_pyd_);
        }
    private:
        Location &parent_;
    };

    class CXYProxy {
    public:
        CXYProxy(Location &parent) : parent_(parent) {}
        CXYProxy& operator=(const CXYD &value) {
            if(!solverRegistered_) {
                roslog::error("Solver not registered. CXY assignment ignored.");
                return *this;
            }
            // 利用CXY2PYD转换后更新基本变量
            parent_.base_pyd_ = solverInstance_->CXYD2PYD(value);
            parent_.updateFused();
            return *this;
        }
        operator CXYD() const {
            if(!solverRegistered_) return CXYD{0,0};
            return solverInstance_->PYD2CXYD(parent_.base_pyd_);
        }
    private:
        Location &parent_;
    };

    // 融合数据代理类（带imu融合）：pyd_imu、xyz_imu、cxy_imu
    class PYDIMUProxy {
    public:
        PYDIMUProxy(Location &parent) : parent_(parent) {}
        PYDIMUProxy& operator=(const PYD &value) {
            if(!solverRegistered_) {
                roslog::error("Solver not registered. Fused PYD assignment ignored.");
                return *this;
            }
            // 修改融合数据时，通过 separateIMU 还原基础 pyd，再更新融合数据
            parent_.base_pyd_ = solverInstance_->separateIMU(value, parent_.imu);
            parent_.updateFused();
            return *this;
        }
        operator PYD() const { return parent_.fused_pyd_; }
    private:
        Location &parent_;
    };

    class XYZIMUProxy {
    public:
        XYZIMUProxy(Location &parent) : parent_(parent) {}
        XYZIMUProxy& operator=(const XYZ &value) {
            if(!solverRegistered_) {
                roslog::error("Solver not registered. Fused XYZ assignment ignored.");
                return *this;
            }
            // 先转换为 PYD，再通过 separateIMU 更新基础 pyd，进而更新融合数据
            PYD tmp = solverInstance_->XYZ2PYD(value);
            parent_.base_pyd_ = solverInstance_->separateIMU(tmp, parent_.imu);
            parent_.updateFused();
            return *this;
        }
        operator XYZ() const {
            if(!solverRegistered_) return XYZ{0,0,0};
            return solverInstance_->PYD2XYZ(parent_.fused_pyd_);
        }
    private:
        Location &parent_;
    };

    class CXYIMUProxy {
    public:
        CXYIMUProxy(Location &parent) : parent_(parent) {}
        CXYIMUProxy& operator=(const CXYD &value) {
            if(!solverRegistered_) {
                roslog::error("Solver not registered. Fused CXY assignment ignored.");
                return *this;
            }
            PYD tmp = solverInstance_->CXYD2PYD(value);
            parent_.base_pyd_ = solverInstance_->separateIMU(tmp, parent_.imu);
            parent_.updateFused();
            return *this;
        }
        operator CXYD() const {
            if(!solverRegistered_) return CXYD{0,0};
            return solverInstance_->PYD2CXYD(parent_.fused_pyd_);
        }
    private:
        Location &parent_;
    };

    // 构造函数初始化所有代理对象
    Location() : 
        pyd(*this), xyz(*this), cxy(*this),
        pyd_imu(*this), xyz_imu(*this), cxy_imu(*this)
    {}
    // 自定义拷贝构造函数：复制内部数据，并重新初始化代理成员
    Location(const Location &other)
        : base_pyd_(other.base_pyd_),
          fused_pyd_(other.fused_pyd_),
          imu(other.imu),
          pyd(*this), xyz(*this), cxy(*this),
          pyd_imu(*this), xyz_imu(*this), cxy_imu(*this)
    {}

    // 自定义赋值运算符：只复制内部数据，代理成员保持不变
    Location& operator=(const Location &other){
        if(this != &other) {
            base_pyd_ = other.base_pyd_;
            fused_pyd_ = other.fused_pyd_;
            imu = other.imu;
            updateFused();
        }
        return *this;
    }

    // 对外公开接口，直接读写
    PYDProxy pyd;         // 非融合的 pyd
    XYZProxy xyz;         // 非融合的 xyz，从 pyd 转换得出
    CXYProxy cxy;         // 非融合的 cxy，从 pyd 转换得出

    PYDIMUProxy pyd_imu;  // 融合后的 pyd
    XYZIMUProxy xyz_imu;  // 融合后的 xyz，从 fused pyd 转换得出
    CXYIMUProxy cxy_imu;  // 融合后的 cxy，从 fused pyd 转换得出
    PYD getOrgPYD(const PYD& imu_) const {
        if(!solverRegistered_) {
            roslog::error("Solver not registered. getPYDwithIMU failed.");
            return PYD{0,0,0};
        }
        return solverInstance_->separateIMU(fused_pyd_, imu_);
    }
    
    XYZ getOrgXYZ(const PYD& imu_) const {
        if(!solverRegistered_) {
            roslog::error("Solver not registered. getXYZwithIMU failed.");
            return XYZ{0,0,0};
        }
        return solverInstance_->PYD2XYZ(getOrgPYD(imu_));
    }
    
    CXYD getOrgCXY(const PYD& imu_) const {
        if(!solverRegistered_) {
            roslog::error("Solver not registered. getCXYwithIMU failed.");
            return CXYD{0,0};
        }
        return solverInstance_->PYD2CXYD(getOrgPYD(imu_));
    }
    
    PYD getImuPYD(const PYD& imu_) const {
        if(!solverRegistered_) {
            roslog::error("Solver not registered. getPYDwithIMU failed.");
            return PYD{0,0,0};
        }
        return solverInstance_->fuseIMU(base_pyd_, imu_);
    }
    
    XYZ getImuXYZ(const PYD& imu_) const {
        if(!solverRegistered_) {
            roslog::error("Solver not registered. getXYZwithIMU failed.");
            return XYZ{0,0,0};
        }
        return solverInstance_->PYD2XYZ(getImuPYD(imu_));
    }
    
    CXYD getImuCXY(const PYD& imu_) const {
        if(!solverRegistered_) {
            roslog::error("Solver not registered. getCXYwithIMU failed.");
            return CXYD{0,0};
        }
        return solverInstance_->PYD2CXYD(getImuPYD(imu_));
    }

    PYD imu;         // imu 数据

private:
    // 更新融合数据：由基础 pyd 与 imu 融合得到 fused_pyd
    void updateFused() {
        if(!solverRegistered_) {
            roslog::error("Solver not registered. updateFused failed.");
            return;
        }
        fused_pyd_ = solverInstance_->fuseIMU(base_pyd_, imu);
    }

    // 内部数据存储：以 base_pyd_ 作为非融合基本数据
    PYD base_pyd_;
    // 融合数据，依据 base_pyd_ 与 imu 更新
    PYD fused_pyd_;

    // 静态全局变量：共享的 Solver 接口指针及注册状态（使用shared_ptr管理）
    inline static std::shared_ptr<const solver::BaseSolver> solverInstance_;
    inline static bool solverRegistered_;
};
}