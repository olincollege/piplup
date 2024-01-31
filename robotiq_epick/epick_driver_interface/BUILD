cc_library(
    name = "epick_driver_interface",
    srcs = [
        "crc_utils.cpp",
        "data_utils.cpp",
        "default_driver.cpp",
        "default_driver_utils.cpp",
        "default_serial.cpp",
    ],
    hdrs = [
        "crc_utils.hpp",
        "data_utils.hpp",
        "default_driver.hpp",
        "default_driver_utils.hpp",
        "default_serial.hpp",
        "driver.hpp",
        "driver_exception.hpp",
        "serial.hpp",
    ],
    copts = ["-Irobotiq_epick"],
    visibility = ["//visibility:public"],
    deps = [
        "//third_party:serial",
    ],
)
