load("@drake//tools/skylark:py.bzl", "py_binary")

py_binary(
    name = "teleop_demo",
    srcs = ["teleop_demo.py"],
    imports = [".."],
    deps = [
        "//station",
        "//common",
    ],
)

py_binary(
    name = "hardware_demo",
    srcs = ["hardware_demo.py"],
    imports = [".."],
    deps = ["//station"],
)

py_binary(
    name = "perception_demo",
    srcs = ["perception_demo.py"],
    imports = [".."],
    deps = [
        "//planning",
        "//station",
        "//common",
    ],
)

py_binary(
    name = "quest_teleop_demo",
    srcs = ["quest_teleop_demo.py"],
    imports = [".."],
    deps = ["//station"],
)

py_binary(
    name = "porosity_demo",
    srcs = ["porosity_demo.py"],
    imports = [".."],
    deps = [
        "//planning",
        "//station",
    ],
)

py_binary(
    name = "planar_grasp_demo",
    srcs = ["planar_grasp_demo.py"],
    imports = [".."],
    deps = [
        "//planning",
        "//station",
    ],
)

py_binary(
    name = "com_demo",
    srcs = ["com_demo.py"],
    imports = [".."],
    deps = [
        "//planning",
        "//station",
    ],
)