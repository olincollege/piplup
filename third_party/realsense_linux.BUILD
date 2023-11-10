licenses(["notice"])  # BSD license

exports_files(["LICENSE"])

cc_library(
    name = "realsense",
    srcs = glob(
        [
            "lib/x86_64-linux-gnu/librealsense2-gl.so",
            "lib/x86_64-linux-gnu/librealsense2-gl.so.2.40",
            "lib/x86_64-linux-gnu/librealsense2-gl.so.2.40.0",
            "lib/x86_64-linux-gnu/librealsense2.so",
            "lib/x86_64-linux-gnu/librealsense2.so.2.40",
            "lib/x86_64-linux-gnu/librealsense2.so.2.40.0",
            "lib/x86_64-linux-gnu/librealsense-file.a",
        ],
    ),
    hdrs = glob([
        "include/librealsense2/*.h*",
        "include/librealsense2/**/*.h*",
        "include/librealsense2-gl/**/*.h*",
    ]),
    includes = [
        "include/",
    ],
    linkstatic = 1,
    visibility = ["//visibility:public"],
)