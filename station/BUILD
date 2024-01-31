py_library(
    name = "station",
    srcs = [
        "__init__.py",
        "scenario.py",
        "station.py",
        "gamepad_teleop_controller.py",
        "quest_teleop_controller.py",
    ],
    data = ["//models"],
    imports = [".."],
    visibility = ["//visibility:public"],
    deps = [
        "//common",
        "//kinova_gen3",
        "//robotiq_2f_85",
        "//robotiq_epick",
        "//robotiq_epick:epick_interface_py",
        "//realsense:realsensed400_py",
        "//uss_dbs",
        "//perception",
        "@drake//bindings/pydrake",
    ],
)
