FROM osrf/ros:jazzy-desktop-full-noble
SHELL ["/bin/bash", "-lc"]

# Add ROS2 sources to bashrc
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc

COPY assets/ArenaViewMP*_Linux_x64*.tar.gz /tmp/
RUN set -eux; \
    shopt -s nullglob; \
    AVMP_FILES=(/tmp/ArenaViewMP*_Linux_x64*.tar.gz); \
    if [ "${#AVMP_FILES[@]}" -eq 0 ]; then \
      echo "ERROR: No ArenaViewMP*_Linux_x64*.tar.gz found under assets/ (build context)."; \
      exit 1; \
    fi; \
    if [ "${#AVMP_FILES[@]}" -gt 1 ]; then \
      echo "ERROR: Multiple AVMP tarballs found. Keep only one in assets/:"; \
      printf '  - %s\n' "${AVMP_FILES[@]}"; \
      exit 1; \
    fi; \
    mv "${AVMP_FILES[0]}" /tmp/avmp.tar.gz

RUN apt-get update && apt-get install -y --no-install-recommends \
    ca-certificates \
    tar \
    bash \
    kmod \
    iproute2 \
    ethtool \
    pciutils \
    dkms \
    build-essential \
  && rm -rf /var/lib/apt/lists/*


# Extract AVMP
RUN set -eux; \
    mkdir -p /opt/avmp; \
    tar -xzf /tmp/avmp.tar.gz -C /opt/avmp; \
    rm -f /tmp/avmp.tar.gz; \
    test -d /opt/avmp/ArenaSDK_Linux_x64; \
    test -f /opt/avmp/Lucid_Broadcom_Driver_Package_v*.tar.gz; \
    ln -sfn /opt/avmp/ArenaSDK_Linux_x64 /opt/ArenaSDK_Linux_x64

# Extract nested Broadcom driver package
RUN set -eux; \
    mkdir -p /opt/broadcom; \
    tar -xzf /opt/avmp/Lucid_Broadcom_Driver_Package_v*.tar.gz -C /opt/broadcom; \
    ls -la /opt/broadcom

# Run the Arena SDK configuration script at build time
RUN set -eux; \
    cd /opt/ArenaSDK_Linux_x64; \
    chmod +x ./Arena_SDK_Linux_x64_AVMP.conf; \
    export DEBIAN_FRONTEND=noninteractive; \
    printf 'n\n' | sh ./Arena_SDK_Linux_x64_AVMP.conf -r -cti; \
    ldconfig; \
    chmod +x ./OutputDirectory/Linux/x64Release/avmp_run.sh || true; \
    chmod +x ./OutputDirectory/Linux/x64Release/CefViewWing || true; \
    find ./OutputDirectory/Linux/x64Release -maxdepth 1 -type f -name "*.sh" -exec chmod +x {} \; || true

RUN cat >/usr/local/bin/avmp <<'EOF' && chmod +x /usr/local/bin/avmp
#!/usr/bin/env bash
set -euo pipefail
cd /opt/ArenaSDK_Linux_x64/OutputDirectory/Linux/x64Release
exec ./avmp_run.sh "$@"
EOF

RUN cat >/usr/local/bin/install_broadcom_driver <<'EOF' && chmod +x /usr/local/bin/install_broadcom_driver
#!/usr/bin/env bash
set -euo pipefail

# Requires: --privileged and host mounts of /lib/modules and /usr/src (and usually /var/lib/dkms)
KVER="$(uname -r)"
if [ ! -e "/lib/modules/${KVER}/build" ]; then
  echo "ERROR: Host kernel headers not found at /lib/modules/${KVER}/build"
  echo "Install on the host: sudo apt-get install -y linux-headers-${KVER}"
  exit 1
fi

PKG_DIR="$(find /opt/broadcom -maxdepth 2 -type d -name 'Lucid_Broadcom_Driver_Package*' -print -quit || true)"
if [ -z "$PKG_DIR" ]; then
  echo "ERROR: Could not find Lucid_Broadcom_Driver_Package directory under /opt/broadcom"
  exit 1
fi

SCRIPT="${PKG_DIR}/install_broadcom_driver.sh"
if [ ! -f "$SCRIPT" ]; then
  echo "ERROR: install_broadcom_driver.sh not found at: $SCRIPT"
  exit 1
fi

chmod +x "$SCRIPT" || true

echo "Running Broadcom installer from: $SCRIPT"
echo "NOTE: This modifies the HOST kernel (modules/drivers)."
cd "$PKG_DIR"
exec bash "$SCRIPT"
EOF

# ROS2 installs
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-rclcpp-components \
  && rm -rf /var/lib/apt/lists/*

CMD ["bash"]
