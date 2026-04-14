#!/bin/bash
# Usage: ./copy_to_usb.sh <bag_name>
# Example: ./copy_to_usb.sh racetrack_left_with_signs_04-05_09-50
# Or with env var: export BAG=racetrack_left; ./copy_to_usb.sh $BAG
#
# --- Manual steps (what this script does) ---
# 1. Find USB device:
#      lsblk
#    Look for a disk with RM=1 (removable), usually /dev/sda1
#
# 2. Mount it:
#      mkdir -p /mnt/usb
#      mount /dev/sda1 /mnt/usb
#
# 3. Copy files:
#      cp /mxck2_ws/src/mxck_run/bagfiles/<name>/*.mcap /mnt/usb/
#      cp -r /mxck2_ws/src/mxck_run/export/<name> /mnt/usb/
#
# 4. Unmount before unplugging:
#      umount /mnt/usb
# --------------------------------------------

set -e  # exit on any error

BAG=${1:-$BAG}  # use argument, fall back to $BAG env variable

# --- Config ---
BAG_SRC="/mxck2_ws/src/mxck_run/bagfiles/$BAG"
EXPORT_SRC="/mxck2_ws/src/mxck_run/export/$BAG"
USB_DEV="/dev/sda1"
MOUNT_POINT="/mnt/usb"

# --- Checks ---
if [ -z "$BAG" ]; then
    echo "Error: No bag name provided."
    echo "Usage: ./copy_to_usb.sh <bag_name>"
    echo "   or: export BAG=<bag_name> && ./copy_to_usb.sh"
    exit 1
fi

if [ ! -e "$USB_DEV" ]; then
    echo "Error: USB device $USB_DEV not found. Is it plugged in?"
    exit 1
fi

# --- Mount ---
mkdir -p "$MOUNT_POINT"
mount "$USB_DEV" "$MOUNT_POINT"
echo "Mounted $USB_DEV to $MOUNT_POINT"

# --- Copy ---

# 1. Copy .mcap file (primary)
MCAP_FILE=$(find "$BAG_SRC" -name "*.mcap" 2>/dev/null | head -1)
if [ -n "$MCAP_FILE" ]; then
    echo "Copying mcap file..."
    cp "$MCAP_FILE" "$MOUNT_POINT/"
    echo "  Done: $MCAP_FILE"
else
    echo "  Warning: No .mcap file found in $BAG_SRC, skipping."
fi

# 2. Copy exports (images, video)
if [ -d "$EXPORT_SRC" ]; then
    echo "Copying exports..."
    cp -r "$EXPORT_SRC" "$MOUNT_POINT/"
    echo "  Done: $EXPORT_SRC"
else
    echo "  Warning: No export directory found at $EXPORT_SRC, skipping."
fi

# --- Unmount ---
umount "$MOUNT_POINT"
echo ""
echo "Done! USB unmounted — safe to unplug."