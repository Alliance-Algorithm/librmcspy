#include <atomic>
#include <new>
#include <thread>

#include <pybind11/functional.h>
#include <pybind11/pybind11.h>
#include <pybind11/pytypes.h>

#include <librmcs/forwarder/cboard.hpp>

namespace py = pybind11;

struct PyUtilityObjectCollection {
    py::module_ inspect = py::module_::import("inspect");

    py::function inspect_signature = inspect.attr("signature");
    py::object inspect_empty = inspect.attr("_empty");

    py::module_ asyncio = py::module_::import("asyncio");
    py::function is_coroutine_function = py::module_::import("inspect").attr("iscoroutinefunction");

    py::object asyncio_event_loop = asyncio.attr("get_event_loop")();
    py::function run_coroutine_threadsafe = asyncio.attr("run_coroutine_threadsafe");
};

template <typename... Args>
class Delegate {
public:
    constexpr explicit Delegate(const std::array<const char*, sizeof...(Args)>& arg_names)
        : arg_names_{arg_names} {}

    ~Delegate() {
        auto call_struct = linked_call_struct_.load(std::memory_order::relaxed);
        while (call_struct) {
            auto next_call_struct = call_struct->next.load(std::memory_order::relaxed);
            delete call_struct;
            call_struct = next_call_struct;
        }
    }

    void add(const PyUtilityObjectCollection& util, const py::function& callable) {
        auto new_call_struct = new LinkedCallStruct{
            nullptr, callable, util.is_coroutine_function(callable).template cast<bool>(), {}};

        auto para = util.inspect_signature(callable).attr("parameters").template cast<py::dict>();
        for (size_t i = 0; i < sizeof...(Args); i++) {
            if (para.contains(arg_names_[i])) {
                para.attr("pop")(arg_names_[i]);
                new_call_struct->arg_required[i] = true;
            }
        }
        for (const auto& [name, prop] : para) {
            if (prop.attr("default").template cast<py::object>().is(util.inspect_empty)) {
                throw std::invalid_argument{
                    (callable.attr("__name__").template cast<std::string>()
                     + std::string{"() has an unexpected argument '"}
                     + name.template cast<std::string>() + "'")};
            }
        }

        auto call_struct = linked_call_struct_.load(std::memory_order::relaxed);
        if (!call_struct) {
            linked_call_struct_.store(new_call_struct, std::memory_order::relaxed);
        } else {
            auto last_call_struct = call_struct;
            while (auto next = last_call_struct->next.load(std::memory_order::relaxed))
                last_call_struct = next;
            last_call_struct->next.store(new_call_struct, std::memory_order::relaxed);
        }
    }

    void call(const PyUtilityObjectCollection& util, const Args&... args) {
        py::tuple arg_tuple = py::make_tuple(args...);

        for (auto call_struct = linked_call_struct_.load(std::memory_order::relaxed); call_struct;
             call_struct = call_struct->next.load(std::memory_order::relaxed)) {

            py::dict kwargs;
            for (size_t i = 0; i < sizeof...(Args); i++) {
                if (call_struct->arg_required[i])
                    kwargs[arg_names_[i]] = arg_tuple[i];
            }

            if (call_struct->is_async) {
                auto awaitable = call_struct->function(**kwargs);
                util.run_coroutine_threadsafe(awaitable, util.asyncio_event_loop);
            } else {
                call_struct->function(**kwargs);
            }
        }
    }

    explicit operator bool() { return linked_call_struct_.load(std::memory_order::relaxed); }

private:
    struct LinkedCallStruct {
        std::atomic<LinkedCallStruct*> next;
        py::function function;
        bool is_async;
        std::array<bool, sizeof...(Args)> arg_required;
    };

    std::array<const char*, sizeof...(Args)> arg_names_;
    std::atomic<LinkedCallStruct*> linked_call_struct_{nullptr};
};

class CBoard : librmcs::forwarder::CBoard {
public:
    explicit CBoard(uint16_t usb_pid)
        : librmcs::forwarder::CBoard(usb_pid)
        , event_thread_([this]() { handle_events(); }) {
        event_thread_.detach();
    }

    py::function can1_receive(const py::function& callable) {
        can1_receive_callback_.add(util_objects_, callable);
        return callable;
    }
    py::function can2_receive(const py::function& callable) {
        can2_receive_callback_.add(util_objects_, callable);
        return callable;
    }

    py::function uart1_receive(const py::function& callable) {
        uart1_receive_callback_.add(util_objects_, callable);
        return callable;
    }
    py::function uart2_receive(const py::function& callable) {
        uart2_receive_callback_.add(util_objects_, callable);
        return callable;
    }
    py::function dbus_receive(const py::function& callable) {
        dbus_receive_callback_.add(util_objects_, callable);
        return callable;
    }

    py::function accelerometer_receive(const py::function& callable) {
        accelerometer_receive_callback_.add(util_objects_, callable);
        return callable;
    }
    py::function gyroscope_receive(const py::function& callable) {
        gyroscope_receive_callback_.add(util_objects_, callable);
        return callable;
    }

private:
    void can1_receive_callback(
        uint32_t can_id, uint64_t can_data, bool is_extended_can_id, bool is_remote_transmission,
        uint8_t can_data_length) override {
        if (!can1_receive_callback_)
            return;

        py::gil_scoped_acquire acquire;
        auto can_bytes = std::launder(reinterpret_cast<const char*>(&can_data));
        can1_receive_callback_.call(
            util_objects_, can_id, py::bytes{can_bytes, can_data_length}, is_extended_can_id,
            is_remote_transmission);
    }
    void can2_receive_callback(
        uint32_t can_id, uint64_t can_data, bool is_extended_can_id, bool is_remote_transmission,
        uint8_t can_data_length) override {
        if (!can2_receive_callback_)
            return;

        py::gil_scoped_acquire acquire;
        auto can_bytes = std::launder(reinterpret_cast<const char*>(&can_data));
        can2_receive_callback_.call(
            util_objects_, can_id, py::bytes{can_bytes, can_data_length}, is_extended_can_id,
            is_remote_transmission);
    }

    void uart1_receive_callback(const std::byte* uart_data, uint8_t uart_data_length) override {
        if (!uart1_receive_callback_)
            return;

        py::gil_scoped_acquire acquire;
        uart1_receive_callback_.call(
            util_objects_,
            py::bytes{std::launder(reinterpret_cast<const char*>(uart_data)), uart_data_length});
    }
    void uart2_receive_callback(const std::byte* uart_data, uint8_t uart_data_length) override {
        if (!uart2_receive_callback_)
            return;

        py::gil_scoped_acquire acquire;
        uart2_receive_callback_.call(
            util_objects_,
            py::bytes{std::launder(reinterpret_cast<const char*>(uart_data)), uart_data_length});
    }
    void dbus_receive_callback(const std::byte* uart_data, uint8_t uart_data_length) override {
        if (!dbus_receive_callback_)
            return;

        py::gil_scoped_acquire acquire;
        dbus_receive_callback_.call(
            util_objects_,
            py::bytes{std::launder(reinterpret_cast<const char*>(uart_data)), uart_data_length});
    }

    void accelerometer_receive_callback(int16_t x, int16_t y, int16_t z) override {
        if (!accelerometer_receive_callback_)
            return;

        py::gil_scoped_acquire acquire;
        accelerometer_receive_callback_.call(util_objects_, x, y, z);
    }
    void gyroscope_receive_callback(int16_t x, int16_t y, int16_t z) override {
        if (!gyroscope_receive_callback_)
            return;

        py::gil_scoped_acquire acquire;
        gyroscope_receive_callback_.call(util_objects_, x, y, z);
    }

    std::thread event_thread_;

    PyUtilityObjectCollection util_objects_;

    Delegate<uint32_t, py::bytes, bool, bool> can1_receive_callback_{
        {"can_id", "can_data", "is_extended_can_id", "is_remote_transmission"}
    };
    Delegate<uint32_t, py::bytes, bool, bool> can2_receive_callback_{
        {"can_id", "can_data", "is_extended_can_id", "is_remote_transmission"}
    };

    Delegate<py::bytes> uart1_receive_callback_{{"uart_data"}};
    Delegate<py::bytes> uart2_receive_callback_{{"uart_data"}};
    Delegate<py::bytes> dbus_receive_callback_{{"uart_data"}};

    Delegate<int16_t, int16_t, int16_t> accelerometer_receive_callback_{
        {"x", "y", "z"}
    };
    Delegate<int16_t, int16_t, int16_t> gyroscope_receive_callback_{
        {"x", "y", "z"}
    };
};

void spin() { py::module_::import("asyncio").attr("get_event_loop")().attr("run_forever")(); }

PYBIND11_MODULE(__librmcscpp, m) {
    m.attr("__name__") = "librmcspy.__librmcscpp";

    m.doc() = R"pbdoc(
        Cpp part of librmcspy, created by pybind11.
    )pbdoc";

    m.def("spin", &spin);

    py::class_<CBoard>(m, "CBoard")
        .def(py::init<uint16_t>(), py::arg("usb_pid"))
        .def("can1_receive", &CBoard::can1_receive, py::arg("callable"))
        .def("can2_receive", &CBoard::can2_receive, py::arg("callable"))
        .def("uart1_receive", &CBoard::uart1_receive, py::arg("callable"))
        .def("uart2_receive", &CBoard::uart2_receive, py::arg("callable"))
        .def("dbus_receive", &CBoard::dbus_receive, py::arg("callable"))
        .def("accelerometer_receive", &CBoard::accelerometer_receive, py::arg("callable"))
        .def("gyroscope_receive", &CBoard::gyroscope_receive, py::arg("callable"));

#define STRINGIFY(x) #x
#ifdef VERSION_INFO
    m.attr("__version__") = STRINGIFY(VERSION_INFO);
#else
    m.attr("__version__") = "dev";
#endif
}
