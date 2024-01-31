load("@drake//tools/skylark:pybind.bzl", "pybind_py_library")

cc_library(
    name = "suction_force_model",
    srcs = [
        "suction_force_model.cc",
    ],
    hdrs = [
        "suction_force_model.h",
    ],
    deps = [
        "@drake//geometry:scene_graph",
        "@drake//multibody/plant",
        "@drake//systems/framework:leaf_system",
    ],
)

pybind_py_library(
    name = "suction_gripper_py",
    cc_deps = [
        ":suction_force_model",
        "@drake//bindings/pydrake/common:cpp_template_pybind",
        "@drake//bindings/pydrake/common:default_scalars_pybind",
    ],
    cc_so_name = "suction_gripper",
    cc_srcs = ["suction_gripper_py.cc"],
    py_deps = ["@drake//bindings/pydrake"],
    py_imports = ["."],
)

py_library(
    name = "robotiq_epick",
    srcs = [
        "__init__.py",
        "epick_control.py",
        "epick_driver.py",
    ],
    data = ["//models"],
    visibility = ["//visibility:public"],
    deps = [
        ":suction_gripper_py",
        "@drake//bindings/pydrake",
    ],
)

cc_library(
    name = "epick_interface",
    srcs = [
        "epick_interface.cc",
    ],
    hdrs = [
        "epick_interface.h",
    ],
    copts = ["-Irobotiq_epick"],
    deps = [
        "//robotiq_epick/epick_driver_interface",
        "@drake//geometry:scene_graph",
        "@drake//multibody/plant",
        "@drake//systems/analysis:simulator",
        "@drake//systems/framework:leaf_system",
    ],
)

pybind_py_library(
    name = "epick_interface_py",
    cc_copts = ["-Irobotiq_epick"],
    cc_deps = [
        ":epick_interface",
        "@drake//bindings/pydrake/common:cpp_template_pybind",
        "@drake//bindings/pydrake/common:default_scalars_pybind",
        "@drake//bindings/pydrake/common:serialize_pybind",
    ],
    cc_so_name = "epick_interface",
    cc_srcs = [
        "epick_interface_py.cc",
    ],
    py_deps = ["@drake//bindings/pydrake"],
    py_imports = ["."],
    visibility = ["//visibility:public"],
)

cc_binary(
    name = "grip",
    srcs = ["grip.cpp"],
    copts = ["-Irobotiq_epick"],
    deps = [
        "//robotiq_epick/epick_driver_interface",
    ],
)

cc_binary(
    name = "release",
    srcs = ["release.cpp"],
    copts = ["-Irobotiq_epick"],
    deps = [
        "//robotiq_epick/epick_driver_interface",
    ],
)
