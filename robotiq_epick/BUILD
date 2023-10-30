load("@pybind11_bazel//:build_defs.bzl", "pybind_extension")

pybind_extension(
    name = "my_pb_mod",  # This name is not actually created!
    srcs = ["my_pb_mod.cc"],
)

py_library(
    name = "my_pb_mod",
    data = [":my_pb_mod.so"],
)

py_test(
    name = "example_test",
    srcs = ["example_test.py"],
    deps = [
        ":my_pb_mod"
    ],
)

cc_library(
    name = "suction_force_model",
    srcs = [
        "suction_force_model.cc",
    ],
    hdrs = [
        "suction_force_model.h",
    ],
    deps = [
        "@drake//:drake_shared_library",
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
        "@drake//:drake_shared_library",
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
        "@drake//:drake_shared_library",
    ],
)
