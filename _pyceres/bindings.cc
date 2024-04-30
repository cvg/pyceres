#include "_pyceres/core/bindings.h"

#include "_pyceres/factors/bindings.h"
#include "_pyceres/helpers.h"
#include "_pyceres/logging.h"

#include <pybind11/iostream.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/stl_bind.h>

namespace py = pybind11;

PYBIND11_MODULE(pyceres, m) {
  m.doc() = "PyCeres - Python bindings for the Ceres solver.";
  m.attr("__version__") = py::str(VERSION_INFO);
  m.attr("__ceres_version__") = py::str(CERES_VERSION_STRING);

  py::add_ostream_redirect(m, "ostream_redirect");
  BindLogging(m);
  BindCore(m);

  py::module_ f = m.def_submodule("factors");
  BindFactors(f);
}
