# Requirements

To run Adaptio successfully a few requirements must be met.

## PN Driver

The profinet driver requires:

- rt-kernel patches, on NixOS this can be set by changing system kernel: `boot.kernelPackages = pkgs.linuxPackages-rt_latest;`.
- The local (ephemeral) port range must be increased, on NixOS this can be set by `boot.kernel.sysctl = { "net.ipv4.ip_local_port_range" = "49152 60999"; };`.
- Application must be run as root/superuser.
- The interface name must be set in the controller configuration.
- The driver uses a file "rema.xml" to remember profinet name, ip and such. The path is configured in the controller configuration and must be writable by the application.
- The PLC must configure the profinet name and ip of the network interface. If this is not done automatically it must be set via TIA Portal or PRONETA.

## Basler

The Basler camera requires some tuning on the network interface to function optimally, the recommendation for GigE Vision is:

- Enable jumbo frames if possible; the bigger, the better.
- Set packet size to maximum.
- (Linux only) Increase ring buffer size to maximum.
- Enable interrupt moderation and set the interrupt moderation rate to a balanced value.
- Use a dedicated interface that directly connects to the camera.
