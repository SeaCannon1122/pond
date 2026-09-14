#pragma once
#include <cstdint>
#include <optional>

struct MotorCommand
{
    std::optional<double> pos = std::nullopt;
    std::optional<double> vel = std::nullopt;
    std::optional<double> acc = std::nullopt;
    std::optional<double> torque = std::nullopt;
    std::optional<bool> disable = std::nullopt;
};

struct MotorFeedback
{
    double time = 0;
    double hw_time = 0;

    std::optional<double> pos = std::nullopt;
    std::optional<double> vel = std::nullopt;
    std::optional<double> torque = std::nullopt;
    std::optional<double> current = std::nullopt;
    std::optional<double> temperature = std::nullopt;
};