py_library(
    name = "kinova_gen3",
    srcs = [
        "__init__.py",
        "gamepad_diff_ik.py",
        "quest_endeffector_controller.py",
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
