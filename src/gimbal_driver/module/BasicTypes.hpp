// AUTO-COMMENT: file overview
// This file belongs to the ROS2 sentry workspace codebase.
// Keep behavior and interface changes synchronized with related modules.

#pragma once

#include <cstdint>
#include <cstddef>
#include <array>
#include <span>
#include <algorithm>
#include <ranges>

namespace LangYa
{
    using AngleType = float;
    using Angle100Type = std::int16_t;

#pragma pack(push, 1)
    struct GimbalAnglesType
    {
        AngleType Yaw;
        AngleType Pitch;
    };

    struct VelocityType
    {
        std::int8_t X;
        std::int8_t Y;
    };

    struct UWBPositionType
    {
        std::int16_t X;
        std::int16_t Y;
    };

    struct FireCodeType
    {
        /// @brief 开火状态，所有位反转表示开火 0b00 <-> 0b11
        std::uint8_t FireStatus : 2 = 0;

        /// @brief 电容状态， 00:不用 01:轻度使用 10:重度使用
        std::uint8_t CapState : 2 = 0;

        /// @brief 钻洞模式和辐瞄模式, 1表示启用，0表示禁用
        std::uint8_t HoleMode : 1 = 0;
        std::uint8_t AimMode : 1 = 0;

        /// @brief 小陀螺状态，共四档，0 表示无速，1 表示低速， 2 表示中速， 3 表示高速
        std::uint8_t Rotate : 2 = 0;

        /// @brief 翻转开火标志位
        void FlipFireStatus() noexcept { FireStatus = FireStatus == 0 ? 0b11 : 0b00; }
    };

    /// @note 这里类型要求使用 @c std::uint16_t ，是因为在非 g++ 编译器上，此结构体的大小可能不符合预期
    struct GameCodeType
    {
        std::uint16_t IsGameBegin : 1 = 0;
        std::uint16_t HeroPrecaution : 1 = 0;
        std::uint16_t IsMyTeamRed : 1 = 0;
        std::uint16_t EnemyOutpostHealth : 6 = 60;
        std::uint16_t SelfOutpostHealth : 6 = 60;
        std::uint16_t IsReturnedHome : 1 = 0;
    };

    /// 场地事件数据，共32位
    struct ExtEventDataType
    {
        std::uint32_t SelfSupplyStatus : 3 = 0;
        std::uint32_t SelfBuffStatus : 3 = 0;
        std::uint32_t SelfHighlandStatus : 6 = 0;
        std::uint32_t SelfBaseShield : 7 = 0;
        std::uint32_t LastDratTime : 9 = 0;
        std::uint32_t DartTarget : 2 = 0;
        std::uint32_t GainPointstatus : 2 = 0;
    };

    struct GimbalData
    {
        static constexpr auto TypeID = 0;

        GimbalAnglesType GimbalAngles;
        VelocityType Velocity;
        FireCodeType FireCode;
        std::uint8_t CapV;
    };

    struct GameData
    {
        static constexpr auto TypeID = 1;

        GameCodeType GameCode{};
        std::uint16_t AmmoLeft{};
        std::uint16_t TimeLeft{};
        std::uint16_t SelfHealth{};
        std::uint32_t ExtEventData{};
    };

    template<std::size_t TContentSize>
    struct TypedMessage
    {
        std::uint8_t HeadFlag{ '!' };
        std::uint8_t TypeID{ 0 };
        std::array<std::uint8_t, TContentSize> Data;
        std::uint8_t Tail{ 0 };

        template<typename T>
        requires std::is_trivially_copyable_v<T>
        void CopyDataTo(T* object) const noexcept
        {
            static_assert(sizeof(T) == TContentSize, "inconsistent size of data and object");
            std::ranges::copy(this->Data, reinterpret_cast<std::uint8_t*>(object));
        }

        template<typename T>
        requires std::is_trivially_assignable_v<T, const T>
        [[nodiscard]] T& GetDataAs() noexcept
        {
            static_assert(sizeof(T) == TContentSize, "inconsistent size of data and object");
            return *reinterpret_cast<T*>(this->Data.data());
        }

        template<typename T>
        requires std::is_trivially_assignable_v<T, const T>
        [[nodiscard]] const T& GetDataAs() const noexcept
        {
            static_assert(sizeof(T) == TContentSize, "inconsistent size of data and object");
            return *reinterpret_cast<const T*>(this->Data.data());
        }
    };

    static_assert(sizeof(GimbalData) == sizeof(GameData), "inconsistent size of messages");

    struct HealthMyselfData {
        static constexpr auto TypeID = 2;

        std::uint16_t HeroMyself{200};
        std::uint16_t EngineerMyself{};
        std::uint16_t Infantry1Myself{200};
        std::uint16_t Infantry2Myself{};
        std::uint16_t BaseMyself{};
        std::uint16_t SentryMyself{};
    };



    struct HealthEnemyData {
        static constexpr auto TypeID = 3;

        std::uint16_t HeroEnemy{};
        std::uint16_t EngineerEnemy{};
        std::uint16_t Infantry1Enemy{};
        std::uint16_t Infantry2Enemy{};
        std::uint16_t BaseEnemy{}; // 基地血量，分度值100，0-50
        std::uint16_t SentryEnemy{};
    };

    struct GimbalControlData
    {
        std::uint8_t HeadFlag{ '!' };
        VelocityType Velocity;
        GimbalAnglesType GimbalAngles;
        FireCodeType FireCode;
        std::uint8_t Posture{0}; // 0=保留, 1=进攻, 2=防御, 3=移动
        std::uint8_t Tail{ 0 };
    };

    struct BuffType{
        std::uint8_t reserve;
        std::uint8_t RecoveryBuff;
        std::uint8_t CoolingBuff;
        std::uint8_t DefenceBuff;
        std::uint8_t VulnerabilityBuff;
        std::uint16_t AttackBuff;
        std::uint8_t RemainingEnergy;
    };

    struct RFIDAndBuffData{
        static constexpr auto TypeID = 4;

        BuffType BuffStatus;
        std::uint32_t RFIDStatus;
    };

    struct PositionType{
        uint8_t CarId; /// 兵种id
        int16_t X; /// 乘了100
        int16_t Y; 
    };

    struct PositionData{
        static constexpr auto TypeID = 5;

        PositionType Friend;
        PositionType Enemy;
        uint16_t BulletSpeed; // 弹速，100倍
    };


    struct ExtendData {
        static constexpr auto TypeID = 6;
        std::uint16_t UWBAngleYaw; // 2 字节
        std::uint16_t Reserve_16; // 
        std::uint32_t Reserve_32_1;
        std::uint32_t Reserve_32_2;
    };


#pragma pack(pop)
}
