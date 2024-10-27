#pragma once

#include <librmcs/device/dr16.hpp>
#include <pybind11/pybind11.h>
#include <pybind11/pytypes.h>

namespace py = pybind11;

class Dr16 {
public:
    Dr16() = default;

    void update_status(const py::bytes& uart_data) {
        auto view = static_cast<std::string_view>(uart_data);
        dr16_.store_status(reinterpret_cast<const std::byte*>(view.data()), view.size());
        dr16_.update_status();
    }

    py::tuple joystick_right() { return vector_cast(dr16_.joystick_right()); }
    py::tuple joystick_left() { return vector_cast(dr16_.joystick_left()); }

    py::str switch_right() { return switch_cast(dr16_.switch_right()); }
    py::str switch_left() { return switch_cast(dr16_.switch_left()); }

    py::tuple mouse_velocity() { return vector_cast(dr16_.mouse_velocity()); }

    py::dict mouse() {
        auto data = dr16_.mouse();
        return py::dict{
            py::arg("left") = static_cast<bool>(data.left),
            py::arg("right") = static_cast<bool>(data.right)
        };
    }
    py::dict keyboard() {
        auto data = dr16_.keyboard();
        return py::dict{
            py::arg("w") = static_cast<bool>(data.w),
            py::arg("s") = static_cast<bool>(data.s),
            py::arg("a") = static_cast<bool>(data.a),
            py::arg("d") = static_cast<bool>(data.d),
            py::arg("shift") = static_cast<bool>(data.shift),
            py::arg("ctrl") = static_cast<bool>(data.ctrl),
            py::arg("q") = static_cast<bool>(data.q),
            py::arg("e") = static_cast<bool>(data.e),
            py::arg("r") = static_cast<bool>(data.r),
            py::arg("f") = static_cast<bool>(data.f),
            py::arg("g") = static_cast<bool>(data.g),
            py::arg("z") = static_cast<bool>(data.z),
            py::arg("x") = static_cast<bool>(data.x),
            py::arg("c") = static_cast<bool>(data.c),
            py::arg("v") = static_cast<bool>(data.v),
            py::arg("b") = static_cast<bool>(data.b)
        };
    }

private:
    static py::tuple vector_cast(const librmcs::device::Dr16::Vector& vec) {
        return py::make_tuple(vec.x, vec.y);
    }

    static py::str switch_cast(const librmcs::device::Dr16::Switch& sw) {
        switch (sw) {
        case librmcs::device::Dr16::Switch::UNKNOWN: return "unknown";
        case librmcs::device::Dr16::Switch::UP: return "up";
        case librmcs::device::Dr16::Switch::DOWN: return "down";
        case librmcs::device::Dr16::Switch::MIDDLE: return "middle";
        }
    }

    librmcs::device::Dr16 dr16_;
};