#include <stdexcept>

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <pond/manager/manager.hpp>

namespace py = pybind11;

static pond_parameter* make_parameter(const py::handle& value);

static pond_parameter* make_string_parameter(const py::handle& value)
{
    std::string str = value.cast<std::string>();

    // Assumes pond_malloc_parameter_string() COPIES the data.
    return pond_malloc_parameter_string(
        reinterpret_cast<uint8_t*>(str.data())
    );
}

static pond_parameter* make_array_parameter(const py::sequence& seq)
{
    const py::ssize_t length = py::len(seq);

    if (length == 0) {
        throw std::invalid_argument(
            "Empty parameter arrays are not supported"
        );
    }

    // Inspect the first element to determine the array type.
    py::handle first = seq[0];

    // --------------------------------------------------------
    // bool[]
    //
    // Must be checked before int[] because Python bool is an int
    // subclass.
    // --------------------------------------------------------

    if (py::isinstance<py::bool_>(first)) {
        std::vector<bool> values;
        values.reserve(length);

        for (py::ssize_t i = 0; i < length; ++i) {
            py::handle item = seq[i];

            if (!py::isinstance<py::bool_>(item)) {
                throw std::invalid_argument(
                    "Boolean parameter array contains a non-boolean"
                );
            }

            values.push_back(item.cast<bool>());
        }

        // std::vector<bool> is special and does not provide a normal
        // bool* through data(), so create a real bool array.
        std::vector<bool> tmp = values;
        std::unique_ptr<bool[]> native_values(new bool[length]);

        for (py::ssize_t i = 0; i < length; ++i) {
            native_values[i] = tmp[i];
        }

        pond_parameter* result =
            pond_malloc_parameter_bool_array(
                native_values.get(),
                static_cast<uint32_t>(length)
            );

        return result;
    }

    if (py::isinstance<py::int_>(first)) {
        std::vector<int32_t> values;
        values.reserve(length);

        for (py::ssize_t i = 0; i < length; ++i) {
            py::handle item = seq[i];

            // bool is an int in Python, but we don't want
            // [True, False] to become int[].
            if (!py::isinstance<py::int_>(item) ||
                py::isinstance<py::bool_>(item)) {
                throw std::invalid_argument(
                    "Integer parameter array contains a non-integer"
                );
            }

            values.push_back(item.cast<int32_t>());
        }

        return pond_malloc_parameter_int_array(
            values.data(),
            static_cast<uint32_t>(values.size())
        );
    }

    if (py::isinstance<py::float_>(first)) {
        std::vector<double> values;
        values.reserve(length);

        for (py::ssize_t i = 0; i < length; ++i) {
            py::handle item = seq[i];

            if (!py::isinstance<py::float_>(item) &&
                !py::isinstance<py::int_>(item)) {
                throw std::invalid_argument(
                    "Double parameter array contains a non-number"
                );
            }

            if (py::isinstance<py::bool_>(item)) {
                throw std::invalid_argument(
                    "Double parameter array contains a boolean"
                );
            }

            values.push_back(item.cast<double>());
        }

        return pond_malloc_parameter_double_array(
            values.data(),
            static_cast<uint32_t>(values.size())
        );
    }

    if (py::isinstance<py::str>(first)) {
        std::vector<std::string> strings;
        strings.reserve(length);

        for (py::ssize_t i = 0; i < length; ++i) {
            py::handle item = seq[i];

            if (!py::isinstance<py::str>(item)) {
                throw std::invalid_argument(
                    "String parameter array contains a non-string"
                );
            }

            strings.push_back(item.cast<std::string>());
        }

        // Build uint8_t* array.
        std::vector<uint8_t*> pointers;
        pointers.reserve(strings.size());

        for (auto& str : strings) {
            pointers.push_back(
                reinterpret_cast<uint8_t*>(str.data())
            );
        }

        return pond_malloc_parameter_string_array(
            pointers.data(),
            static_cast<uint32_t>(pointers.size())
        );
    }


    throw std::invalid_argument(
        "Unsupported parameter array type"
    );
}

static pond_parameter* make_parameter(const py::handle& value)
{
    // bool must come before int because Python bool is an int

    if (py::isinstance<py::bool_>(value)) return pond_malloc_parameter_bool(value.cast<bool>());
    if (py::isinstance<py::int_>(value)) return pond_malloc_parameter_int(value.cast<int32_t>());
    if (py::isinstance<py::float_>(value)) return pond_malloc_parameter_double(value.cast<double>());
    if (py::isinstance<py::str>(value)) return make_string_parameter(value);


    if (py::isinstance<py::list>(value) || py::isinstance<py::tuple>(value))
        return make_array_parameter(value.cast<py::sequence>());

    throw std::invalid_argument(
        "Unsupported Python parameter type. "
        "Expected bool, int, float, str, list, or tuple."
    );
}

PYBIND11_MODULE(_pond, m)
{
    m.doc() = "Python bindings for pond";

    py::class_<PondManager>(m, "Manager")
        .def(py::init<>())

        .def(
            "load_module",

            [](PondManager& self,
               const std::string& name,
               const std::string& bundle_name,
               const std::string& module_name,
               const std::string& thread_name,
               const py::dict& parameters,
               const std::unordered_map<std::string, std::string>& topic_mappings)
            {
                std::unordered_map<
                    std::string,
                    pond_parameter*
                > native_parameters;

                native_parameters.reserve(
                    parameters.size()
                );


                // Convert Python dict -> C++ unordered_map
                for (auto item : parameters) {
                    std::string key =
                        item.first.cast<std::string>();

                    pond_parameter* parameter =
                        make_parameter(item.second);

                    native_parameters.emplace(
                        std::move(key),
                        parameter
                    );
                }


                return self.load_module(
                    name,
                    bundle_name,
                    module_name,
                    thread_name,
                    native_parameters,
                    topic_mappings,
                    {}
                );
            },

            py::arg("name"),
            py::arg("bundle_name"),
            py::arg("module_name"),
            py::arg("thread_name"),
            py::arg("parameters"),
            py::arg("topic_mappings")
        )

        .def(
            "shutdown_module",
            &PondManager::shutdown_module,
            py::arg("name")
        );
    
}