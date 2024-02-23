load("@drake//tools/skylark:py.bzl", "py_binary")

py_library(
    name = "kinova_gen3",
    srcs = [
        "__init__.py",
        "gen3_constants.py",
        "gen3_control.py",
        "gen3_driver.py",
        "gen3_interface.py",
        "gen3_sim_driver.py",
    ],
    data = ["//models"],
    imports = [".."],
    visibility = ["//visibility:public"],
    deps = [
        "//common",
        "//robotiq_2f_85",
        "@drake//bindings/pydrake",
    ],
)


py_binary(
    name = "interface_demo",
    srcs = ["interface_demo.py"],
    imports = [".."],
    deps = [":kinova_gen3"],
)

cc_library(
    name = "gen3_interface",
    srcs = ["gen3_interface.cc"],
    hdrs = ["gen3_interface.h"],
    deps = [
        "//third_party:kortex",
        "@drake//common:name_value",
        "@drake//common:scoped_singleton",
        "@drake//multibody/plant",
        "@drake//systems/analysis:simulator",
        "@drake//systems/framework:leaf_system",
    ],
)
