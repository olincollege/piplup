workspace(name = "piplup")

# This Bazel WORKSPACE file is taken from the drake-external-examples https://github.com/RobotLocomotion/drake-external-examples/tree/main/drake_bazel_installed
DRAKE_COMMIT = "v1.22.0"

DRAKE_CHECKSUM = "78cf62c177c41f8415ade172c1e6eb270db619f07c4b043d5148e1f35be8da09"

# Load an environment variable.
load("//:environ.bzl", "environ_repository")

environ_repository(
    name = "environ",
    vars = ["EXAMPLES_LOCAL_DRAKE_PATH"],
)

load("@environ//:environ.bzl", "EXAMPLES_LOCAL_DRAKE_PATH")

# This declares the `@drake` repository as an http_archive from github,
# iff EXAMPLES_LOCAL_DRAKE_PATH is unset.  When it is set, this declares a
# `@drake_ignored` package which is never referenced, and thus is ignored.
load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

http_archive(
    name = "drake" if not EXAMPLES_LOCAL_DRAKE_PATH else "drake_ignored",
    sha256 = DRAKE_CHECKSUM,
    strip_prefix = "drake-{}".format(DRAKE_COMMIT.lstrip("v")),
    urls = [x.format(DRAKE_COMMIT) for x in [
        "https://github.com/RobotLocomotion/drake/archive/{}.tar.gz",
    ]],
)

# This declares the `@drake` repository as a local directory,
# iff EXAMPLES_LOCAL_DRAKE_PATH is set.  When it is unset, this declares a
# `@drake_ignored` package which is never referenced, and thus is ignored.
local_repository(
    name = "drake" if EXAMPLES_LOCAL_DRAKE_PATH else "drake_ignored",
    path = EXAMPLES_LOCAL_DRAKE_PATH,
)

print("Using EXAMPLES_LOCAL_DRAKE_PATH={}".format(EXAMPLES_LOCAL_DRAKE_PATH)) if EXAMPLES_LOCAL_DRAKE_PATH else None  # noqa

# Reference external software libraries, tools, and toolchains per Drake's
# defaults.  Some software will come from the host system (Ubuntu or macOS);
# other software will be downloaded in source or binary form from GitHub or
# other sites.
load("@drake//tools/workspace:default.bzl", "add_default_workspace")

add_default_workspace()

new_local_repository(
    name = "linux_realsense",
    build_file = "@//third_party:realsense_linux.BUILD",
    path = "/usr",
)

new_local_repository(
    name = "linux_opencv",
    build_file = "@//third_party:opencv_linux.BUILD",
    path = "/usr",
)

new_local_repository(
    name = "libserial",
    path = "/usr/local",
    build_file = "@//third_party:libserial.BUILD"
)