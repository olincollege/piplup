workspace(name = "piplup")

# This Bazel WORKSPACE file is taken from the drake-external-examples https://github.com/RobotLocomotion/drake-external-examples/tree/main/drake_bazel_installed

load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

http_archive(
    name = "rules_python",
    sha256 = "aa96a691d3a8177f3215b14b0edc9641787abaaa30363a080165d06ab65e1161",
    url = "https://github.com/bazelbuild/rules_python/releases/download/0.0.1/rules_python-0.0.1.tar.gz",
)

load("@rules_python//python:repositories.bzl", "py_repositories")

py_repositories()

# Choose which nightly build of Drake to use.
DRAKE_RELEASE = "latest"  # Can also use YYYYMMDD here, e.g., "20191026".

DRAKE_CHECKSUM = ""  # When using YYYYMMDD, best to add a checksum here.

OS_CODENAME = "focal"  # Permitted values are "focal" or "mac".

# To use a local unpacked Drake binary release instead of an http download, set
# this variable to the correct path, e.g., "/opt/drake".
INSTALLED_DRAKE_DIR = "/opt/drake"

# This is only relevant when INSTALLED_DRAKE_DIR is set.
new_local_repository(
    name = "drake_artifacts",
    build_file_content = "#",
    path = INSTALLED_DRAKE_DIR,
) if INSTALLED_DRAKE_DIR else None

# This is only relevant when INSTALLED_DRAKE_DIR is unset.
http_archive(
    name = "drake_artifacts",
    build_file_content = "#",
    sha256 = DRAKE_CHECKSUM,
    strip_prefix = "drake/",
    url = "https://drake-packages.csail.mit.edu/drake/nightly/drake-{}-{}.tar.gz".format(DRAKE_RELEASE, OS_CODENAME),
) if not INSTALLED_DRAKE_DIR else None

# Load and run the repository rule that knows how to provide the @drake
# repository based on a Drake binary release.
load("@drake_artifacts//:share/drake/repo.bzl", "drake_repository")

drake_repository(name = "drake")
