load("@drake//tools/skylark:py.bzl", "py_binary")
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

cc_library(
    name = "suction_gripper_multibody_model",
    srcs = [
        "suction_gripper_multibody_model.cc",
    ],
    hdrs = [
        "suction_gripper_multibody_model.h",
    ],
    deps = [
        "@drake//geometry:scene_graph",
        "@drake//multibody/plant",
        "@drake//multibody/tree",
    ],
)

cc_library(
    name = "example_gripper_multibody_model",
    srcs = [
        "example_gripper_multibody_model.cc",
    ],
    hdrs = [
        "example_gripper_multibody_model.h",
    ],
    deps = [
        ":suction_gripper_multibody_model",
    ],
)

cc_binary(
    name = "package_swing",
    srcs = ["package_swing.cc"],
    deps = [
        ":example_gripper_multibody_model",
        ":suction_force_model",
        ":suction_gripper_multibody_model",
        "@drake//geometry:drake_visualizer",
        "@drake//multibody/plant",
        "@drake//multibody/tree",
        "@drake//systems/analysis:simulator",
        "@drake//systems/framework:diagram",
        "@drake//systems/framework:diagram_builder",
        "@drake//visualization:visualization_config_functions",
    ],
)

pybind_py_library(
    name = "suction_gripper_py",
    cc_deps = [
        ":example_gripper_multibody_model",
        ":suction_force_model",
        ":suction_gripper_multibody_model",
        "@drake//bindings/pydrake/common:cpp_template_pybind",
        "@drake//bindings/pydrake/common:default_scalars_pybind",
    ],
    cc_so_name = "suction_gripper",
    cc_srcs = ["suction_gripper_py.cc"],
    py_deps = ["@drake//bindings/pydrake"],
    py_imports = ["."],
)

py_binary(
    name = "package_swing_py",
    srcs = ["package_swing_py.py"],
    deps = [":suction_gripper_py"],
)

py_library(
    name = "robotiq_epick",
    srcs = [
        "__init__.py",
        "epick_driver.py",
        "epick_control.py",
    ],
    data = ["//models"],
    visibility = ["//visibility:public"],
    deps = [
        ":suction_gripper_py",
        "@drake//bindings/pydrake",
    ],
)
