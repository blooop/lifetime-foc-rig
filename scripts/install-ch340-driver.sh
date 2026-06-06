#!/usr/bin/env bash
# Install / verify the WCH CH340 (CH34x) USB-serial driver so the MKS ESP32 FOC board
# enumerates as a serial port.
#
#   Linux  : the ch341 driver is built into the kernel — nothing to install.
#   macOS  : 10.14+ ships a built-in CH34x driver that usually "just works"; if the
#            port doesn't appear, this installs WCH's CH34xVCPDriver via Homebrew, which
#            you then APPROVE in System Settings and replug the board.
#
# Run via:  pixi run install-driver   (or  bash scripts/install-ch340-driver.sh )
set -uo pipefail

os="$(uname -s)"

# ---- Linux: driver is in-kernel ----------------------------------------------------
if [ "$os" = "Linux" ]; then
  echo "Linux: the CH340 (ch341) driver is built into the kernel — no install needed."
  echo
  echo "If the board doesn't show up:"
  echo "  • use a real DATA USB cable (not charge-only)"
  echo "  • check it enumerated:   ls /dev/ttyUSB*   and   dmesg | grep -i ch341"
  echo "  • grant serial access:   sudo usermod -aG dialout \"\$USER\"   (then log out/in)"
  exit 0
fi

# ---- not macOS ---------------------------------------------------------------------
if [ "$os" != "Darwin" ]; then
  echo "Unsupported OS: $os (this project targets Linux and macOS)." >&2
  exit 1
fi

# ---- macOS -------------------------------------------------------------------------
echo "macOS $(sw_vers -productVersion 2>/dev/null || echo '?') detected."
echo

# 1. Is a CH340-style serial port already present? (10.14+ has a built-in driver)
existing="$(ls /dev/cu.usbserial-* /dev/cu.wchusbserial* /dev/tty.usbserial-* /dev/tty.wchusbserial* 2>/dev/null || true)"
if [ -n "$existing" ]; then
  echo "A USB-serial port is already present:"
  echo "$existing" | sed 's/^/    /'
  echo "The built-in driver appears to work — you can probably skip the install and just"
  echo "run 'pixi run flash'. Install the WCH driver below only if flashing fails."
  echo
fi

# 2. Install the WCH driver via Homebrew if available
if command -v brew >/dev/null 2>&1; then
  echo "Installing the WCH CH34x driver via Homebrew…"
  if brew install --cask wch-ch34x-usb-serial-driver; then
    echo "Homebrew install OK."
  else
    echo "Homebrew install failed — use the manual download below." >&2
  fi
else
  echo "Homebrew not found. Install it (https://brew.sh) and re-run, or use the manual"
  echo "download below."
fi

cat <<'NOTE'

Next steps (required for a freshly installed driver):
  1. Approve the driver: System Settings > Privacy & Security — click "Allow" for the
     WCH / CH34x system extension. On Apple Silicon / macOS 11+ a reboot may be needed.
  2. Replug the board with a real DATA USB cable.
  3. Confirm the port:
        ls /dev/cu.*usbserial* /dev/cu.*wchusbserial* 2>/dev/null
  4. Use it:   pixi run flash    (or  pixi run panel ).
     The tools auto-detect the port; override with  FOC_PORT=/dev/tty.usbserial-XXXX.

Manual download (if Homebrew isn't available):
  https://github.com/WCHSoftGroup/ch34xser_macos   (official WCH macOS driver)
NOTE
