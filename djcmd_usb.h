/*
 * djcmd_usb.h -- USB export / eject support for djcmd
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#pragma once
#include <stddef.h>
#include "djcmd_shared.h"

/* ── Directory / file names on the USB ──────────────────────────────── */
#define DJCMD_USB_DIR "djcmd-usb"
#define DJCMD_USB_MARKER "djcmd-usb/.djcmd-usb"
#define DJCMD_USB_CRATES "djcmd-usb/crates"
#define DJCMD_USB_MUSIC "djcmd-usb/music"

/* ── .crate file header (v2) ────────────────────────────────────────── */
#define DJCMD_CRATE_HDR "# djcmd-crate-v2"

/*
 * Populate `out` with the machine's stable identifier.
 * Reads /etc/machine-id first; falls back to ~/.config/djcmd/machine_id
 * (generating and persisting a new ID if neither exists).
 */
void usb_get_machine_id(char *out, size_t sz);

/*
 * Scan /proc/mounts for djcmd USB devices (those containing the
 * DJCMD_USB_MARKER file).  Returns the number of devices found (≤ max).
 */
int usb_scan(USBDevice *out, int max);

/*
 * Read the "# origin: <id>" header from a .crate file into `out`.
 * Writes an empty string if the header is absent or the file cannot be read.
 */
void usb_crate_read_origin(const char *crate_path, char *out, size_t sz);

/*
 * Export a local crate to a USB device.
 *
 * local_crate_path  — path to the source .crate file
 * local_crate_name  — display name (used as status prefix)
 * local_origin      — this machine's ID (written into the USB .crate header)
 * usb_mount         — USB mount point
 * export_name       — name for the crate on the USB (may differ if renamed
 *                     to resolve a conflict)
 * status_out        — human-readable result string
 *
 * Audio files are copied to  <usb_mount>/djcmd-usb/music/<abs-path-minus-/>
 * Sidecars are copied next to their audio files (sidecar_path() compatible).
 * Unchanged files (same size + mtime) are skipped.
 *
 * Returns 0 on success, -1 on hard error.
 */
int usb_export_crate(const char *local_crate_path, const char *local_crate_name,
		     const char *local_origin, const char *usb_mount,
		     const char *export_name, char *status_out,
		     size_t status_sz);

/*
 * Unmount and power off a USB device.
 * Returns 0 on success, -1 if umount failed.
 */
int usb_eject(const char *mount_point);
