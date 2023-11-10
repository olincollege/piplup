load("@drake//tools/skylark:pybind.bzl", "pybind_py_library")

cc_library(
    name = "realsensed400",
    srcs = [
        "realsensed400.cc",
    ],
    hdrs = [
        "realsensed400.h",
    ],
    deps = [
        "//third_party:realsense",
        "@drake//common:name_value",
        "@drake//common:scoped_singleton",
        "@drake//systems/analysis:simulator",
        "@drake//systems/framework:leaf_system",
        "@drake//systems/sensors:camera_config",
        "@drake//systems/sensors:image",
    ],
)

cc_binary(
    name = "single_camera_display_test",
    srcs = ["single_camera_display_test.cc"],
    deps = [
        "//third_party:opencv",
        ":realsensed400"
    ],
)

pybind_py_library(
    name = "realsensed400_py",
    cc_deps = [
        ":realsensed400",
        "@drake//bindings/pydrake/common:cpp_template_pybind",
        "@drake//bindings/pydrake/common:default_scalars_pybind",
    ],
    cc_so_name = "realsensed400",
    cc_srcs = ["realsensed400_py.cc"],
    py_deps = ["@drake//bindings/pydrake"],
    py_imports = ["."],
)