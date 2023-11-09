load("@drake//tools/skylark:py.bzl", "py_binary")

py_binary(
    name = "teleop_demo",
    srcs = ["teleop_demo.py"],
    imports = [".."],
    deps = ["//station:station"],
)

py_binary(
    name = "hardware_demo",
    srcs = ["hardware_demo.py"],
    imports = [".."],
    deps = ["//station:station"],
)