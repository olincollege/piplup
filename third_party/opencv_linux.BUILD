licenses(["notice"])  # BSD license

exports_files(["LICENSE"])

cc_library(
    name = "opencv",
    srcs = glob(
        [
            "lib/x86_64-linux-gnu/libopencv*.so",
        ],
    ),
    hdrs = glob([
        "include/opencv4/opencv2/*.h*",
        "include/opencv4/opencv2/**/*.h*",
    ]),
    includes = [
        "include/opencv4",
    ],
    linkstatic = 1,
    visibility = ["//visibility:public"],
)