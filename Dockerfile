FROM ubuntu:latest

ARG DEBIAN_FRONTEND=noninteractive
ADD scripts scripts
RUN ./scripts/setup/install_prereqs.sh -y