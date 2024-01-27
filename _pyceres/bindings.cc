#include "_pyceres/core/bindings.h"

#include "_pyceres/factors/bindings.h"
#include "_pyceres/glog.h"
#include "_pyceres/helpers.h"

#include <pybind11/iostream.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/stl_bind.h>

namespace py = pybind11;

PYBIND11_MODULE(pyceres, m) {
  m.doc() = "PyCeres";

  py::add_ostream_redirect(m, "ostream_redirect");
  init_glog(m);
  bind_core(m);

  py::module_ f = m.def_submodule("factors");
  bind_factors(f);
}
