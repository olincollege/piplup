load("@pybind11_bazel//:build_defs.bzl", "pybind_extension")

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

pybind_extension(
    name = "suction_gripper",  # This name is not actually created!
    srcs = ["suction_gripper.cc"],
    deps = [
        "@drake//:drake_shared_library",
        ":example_gripper_multibody_model",
        ":suction_force_model",
        ":suction_gripper_multibody_model",
    ],
)

py_library(
    name = "suction_gripper",
    data = [":suction_gripper.so"],
)

py_binary(
    name = "package_swing_py",
    main = "package_swing.py",
    srcs = ["package_swing.py"],
    deps = [
        ":suction_gripper",
        "@drake//bindings/pydrake",
    ],
)
