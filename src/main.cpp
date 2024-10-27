#include <pybind11/pybind11.h>

#include "client/cboard.hpp"
#include "device/dr16.hpp"

namespace py = pybind11;

void spin() { py::module_::import("asyncio").attr("get_event_loop")().attr("run_forever")(); }

PYBIND11_MODULE(__librmcscpp, m) {
    m.attr("__name__") = "librmcspy.__librmcscpp";

    m.doc() = R"pbdoc(
        Cpp part of librmcspy, created by pybind11.
    )pbdoc";

    m.def("spin", &spin);

    auto m_client = m.def_submodule("client");

    py::class_<CBoard>(m_client, "CBoard")
        .def(py::init<uint16_t>(), py::arg("usb_pid"))
        .def("can1_receive", &CBoard::can1_receive, py::arg("callable"))
        .def("can2_receive", &CBoard::can2_receive, py::arg("callable"))
        .def("uart1_receive", &CBoard::uart1_receive, py::arg("callable"))
        .def("uart2_receive", &CBoard::uart2_receive, py::arg("callable"))
        .def("dbus_receive", &CBoard::dbus_receive, py::arg("callable"))
        .def("accelerometer_receive", &CBoard::accelerometer_receive, py::arg("callable"))
        .def("gyroscope_receive", &CBoard::gyroscope_receive, py::arg("callable"));

    auto m_device = m.def_submodule("device");

    py::class_<Dr16>(m_device, "Dr16")
        .def(py::init<>())
        .def("update_status", &Dr16::update_status, py::arg("uart_data"))
        .def("joystick_right", &Dr16::joystick_right)
        .def("joystick_left", &Dr16::joystick_left)
        .def("switch_right", &Dr16::switch_right)
        .def("switch_left", &Dr16::switch_left)
        .def("mouse_velocity", &Dr16::mouse_velocity)
        .def("mouse", &Dr16::mouse)
        .def("keyboard", &Dr16::keyboard);

#define STRINGIFY(x) #x
#ifdef VERSION_INFO
    m.attr("__version__") = STRINGIFY(VERSION_INFO);
#else
    m.attr("__version__") = "dev";
#endif
}
